import cv2
import numpy as np
import tensorrt as trt
#import pycuda.autoinit
import pycuda.driver as imgoinginsane
import pyzed.sl as sl
import time

# from projectileMotion import calculateFunction, predictorFunction
from projectileMotion import BallKalman #new kalman filter function


imgoinginsane.init()

device = imgoinginsane.Device(0)

cudaContext = device.retain_primary_context()

cudaContext.push()
ballfilter = BallKalman()


# engine_path = ~/documents/group_random_strangers/software/best.engine  #path to engine file
confidenceIndex = 0.25 
targetIndex = 1 # maybe 2, if we want to attack people lmao
IOUIndex =0.45 

#Tensorrt setup for loading model into memory and creates a model context
logger = trt.Logger(trt.Logger.WARNING)

with open("best.engine", "rb") as f: #open engine
    runtime = trt.Runtime(logger) #runtime init
    engine = runtime.deserialize_cuda_engine(f.read()) #memory load

context = engine.create_execution_context() #context, will open and run this

if engine is None: 
    raise RuntimeError("failed to load engine lmao")


# Tensor Shapes 
intputName = engine.get_tensor_name(0)
outputName = engine.get_tensor_name(1)   #we have an input, balls and humans, and 1 output

inputShape = tuple(engine.get_tensor_shape(intputName)) #(1, 3, 640, 640)
outputShape = tuple(engine.get_tensor_shape(outputName)) #(1, 6, 8400)

print ("TensorRT Input:", inputShape) #prints out 1,3,640,640 etc.
print ("The Output: ", outputShape)

#CPU memory Buffer

hostInput = np.empty(inputShape, dtype=np.float32)
hostOutput = np.empty(outputShape, dtype=np.float32)

#GPU memory buffer
deviceInput = imgoinginsane.mem_alloc(hostInput.nbytes)
deviceOutput = imgoinginsane.mem_alloc(hostOutput.nbytes)

#connect TensorRT to GPU Buffers/address

context.set_tensor_address(intputName, int(deviceInput))
context.set_tensor_address(outputName, int (deviceOutput))

#Create CUDA Stream. AKA Im goinginsane

stream = imgoinginsane.Stream() 

def letterbox(image, size=640):
    h, w = image.shape[:2]

    scale = min(size / w, size / h)

    new_w = int(round(w * scale))
    new_h = int(round(h * scale))

    resized = cv2.resize(
        image,
        (new_w, new_h),
        interpolation=cv2.INTER_LINEAR
    )

    canvas = np.full(
        (size, size, 3),
        114,
        dtype=np.uint8
    )

    pad_x = (size - new_w) // 2
    pad_y = (size - new_h) // 2

    canvas[
        pad_y:pad_y + new_h,
        pad_x:pad_x + new_w
    ] = resized
#code i grabbed that sets the bx bounds to a scaled size.
    return canvas, scale, pad_x, pad_y

def preprocess(frame):

    # 1. Resize/pad the ZED frame to 640x640
    img, scale, pad_x, pad_y = letterbox(frame)

    # 2. OpenCV/ZED gives BGR, YOLO expects RGB
    img = cv2.cvtColor(
        img,
        cv2.COLOR_BGR2RGB
    )

    # 3. Change shape:
    # (height, width, channels)
    #       ↓
    # (channels, height, width)
    #
    # (640, 640, 3)
    #       ↓
    # (3, 640, 640)
    img = img.transpose(
        2,
        0,
        1
    )

    # 4. Convert to float32 and make memory contiguous
    img = np.ascontiguousarray(
        img,
        dtype=np.float32
    )

    # 5. Normalize pixels:
    # 0-255 -> 0.0-1.0
    img /= 255.0

    # 6. Add batch dimension:
    # (3, 640, 640)
    #       ↓
    # (1, 3, 640, 640)
    img = np.expand_dims(
        img,
        axis=0
    )

    # Return the processed image plus the
    # resize information needed later by decode()
    return img, scale, pad_x, pad_y

def infer(input_tensor):

    # Copy processed image into CPU input buffer
    np.copyto(
        hostInput,
        input_tensor
    )

    # Copy CPU input → GPU input
    imgoinginsane.memcpy_htod_async(
        deviceInput,
        hostInput,
        stream
    )

    # Run TensorRT
    context.execute_async_v3(
        stream_handle=stream.handle
    )

    # Copy GPU output toCPU output
    imgoinginsane.memcpy_dtoh_async(
        hostOutput,
        deviceOutput,
        stream
    )

    # Wait for GPU operations to finish
    stream.synchronize()

    # Return independent copy of results
    return hostOutput.copy()


# ============================================================
# DECODE YOLO OUTPUT
# ============================================================

def decode(
    output,
    scale,
    pad_x,
    pad_y,
    frame_w,
    frame_h
):


    predictions = output[0].T


    boxes = []
    scores = []
    class_ids = []


    # ============================================================
    # 1. SPLIT BOX DATA FROM CLASS SCORES
    # ============================================================

    boxes_raw = predictions[:, :4]

    class_scores = predictions[:, 4:]


    # ============================================================
    # 2. FIND BEST CLASS FOR EVERY PREDICTION AT ONCE
    # ============================================================

    class_ids = np.argmax(
        class_scores,
        axis=1
    )

    confidences = np.max(
        class_scores,
        axis=1
    )


    # ============================================================
    # 3. FILTER BY CONFIDENCE AND TARGET CLASS
    # ============================================================

    mask = (
        (confidences >= confidenceIndex)
        &
        (class_ids == targetIndex)
    )


    # Apply the mask to all arrays at once
    boxes_raw = boxes_raw[mask]
    confidences = confidences[mask]
    class_ids = class_ids[mask]


    # No ball detections survived filtering
    if boxes_raw.shape[0] == 0:
        return []

    cx = boxes_raw[:, 0]
    cy = boxes_raw[:, 1]
    w = boxes_raw[:, 2]
    h = boxes_raw[:, 3]


    x1 = cx - w / 2
    y1 = cy - h / 2

    x2 = cx + w / 2
    y2 = cy + h / 2

    x1 = (x1 - pad_x) / scale
    y1 = (y1 - pad_y) / scale

    x2 = (x2 - pad_x) / scale
    y2 = (y2 - pad_y) / scale

    x1 = np.clip(
        x1,
        0,
        frame_w - 1
    )

    y1 = np.clip(
        y1,
        0,
        frame_h - 1
    )

    x2 = np.clip(
        x2,
        0,
        frame_w - 1
    )

    y2 = np.clip(
        y2,
        0,
        frame_h - 1
    )

    boxes = np.column_stack(
        (
            x1,
            y1,
            x2 - x1,
            y2 - y1
        )
    ).astype(np.int32)


    indices = cv2.dnn.NMSBoxes(
        boxes.tolist(),
        confidences.tolist(),
        confidenceIndex,
        IOUIndex
    )


    if len(indices) == 0:
        return []


    # Different OpenCV versions sometimes return different shapes,
    # so flatten it into one simple array.
    indices = np.asarray(
        indices
    ).reshape(-1)


    # ============================================================
    # 9. KEEP ONLY BOXES THAT SURVIVED NMS
    # ============================================================

    boxes = boxes[indices]
    confidences = confidences[indices]
    class_ids = class_ids[indices]


    # ============================================================
    # 10. RETURN SAME FORMAT AS YOUR OLD FUNCTION
    # ============================================================

    detections = [
        {
            "x1": int(box[0]),
            "y1": int(box[1]),
            "x2": int(box[0] + box[2]),
            "y2": int(box[1] + box[3]),
            "confidence": float(confidence),
            "class_id": int(class_id)
        }

        for box, confidence, class_id
        in zip(
            boxes,
            confidences,
            class_ids
        )
    ]


    return detections


# ============================================================
# GET XYZ FROM ZED POINT CLOUD
# ============================================================

def get_xyz(
    point_cloud,
    u,
    v,
    radius=2
):

    samples = []


    # Look at a small region around the center pixel
    for dy in range(
        -radius,
        radius + 1
    ):

        for dx in range(
            -radius,
            radius + 1
        ):

            err, point = point_cloud.get_value(
                u + dx,
                v + dy
            )


            # Ignore invalid ZED measurement
            if err != sl.ERROR_CODE.SUCCESS:
                continue


            xyz = np.array(
                [
                    point[0],
                    point[1],
                    point[2]
                ],
                dtype=np.float32
            )


            # Ignore NaN / Infinity
            if np.all(
                np.isfinite(xyz)
            ):
                samples.append(
                    xyz
                )


    # No usable depth values
    if not samples:
        return None


    # Median helps reject noisy depth pixels
    return np.median(
        np.asarray(samples),
        axis=0
    )
def updateProjectilePhysics(t, x, y, z):

    measurement = (
        x,
        y,
        z
    )

    state = ballfilter.update(
        t,
        measurement
    )

    # First point of a new trajectory only initializes the filter.
    if state is None:
        return None

    filteredX = state[0]
    filteredY = state[1]
    filteredZ = state[2]

    vx = state[3]
    vy = state[4]
    vz = state[5]

    print(
        f"Filtered XYZ=("
        f"{filteredX:.3f}, "
        f"{filteredY:.3f}, "
        f"{filteredZ:.3f}) "
        f"velocity=("
        f"{vx:.3f}, "
        f"{vy:.3f}, "
        f"{vz:.3f}) m/s"
    )

    intercept = ballfilter.getFinalIntercept()

    return intercept
#configure zed cam

# ============================================================
# ZED CAMERA INIT
# ============================================================

zed = sl.Camera()

init = sl.InitParameters()

# Camera mode
init.camera_resolution = sl.RESOLUTION.VGA
init.camera_fps = 60

# Fast depth mode
init.depth_mode = sl.DEPTH_MODE.NEURAL

# XYZ values will be meters
init.coordinate_units = sl.UNIT.METER

# Positive Y points upward
init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP


# ============================================================
# OPEN CAMERA
# ============================================================

status = zed.open(init)

if status != sl.ERROR_CODE.SUCCESS:
    raise RuntimeError(
        f"Could not open ZED: {status}"
    )


# ============================================================
# CREATE REUSABLE ZED OBJECTS
# ============================================================

runtime_params = sl.RuntimeParameters()

zed_image = sl.Mat()

point_cloud = sl.Mat()

print("ZED running")


# ============================================================
# MAIN LOOP
# ============================================================

try:


   while True:


       if (
           zed.grab(runtime_params)
           != sl.ERROR_CODE.SUCCESS
       ):
           continue


       timestamp = zed.get_timestamp(
           sl.TIME_REFERENCE.IMAGE
       )
       t = timestamp.get_nanoseconds() / 1e9


       zed.retrieve_image(
           zed_image,
           sl.VIEW.LEFT
       )


       zed.retrieve_measure(
           point_cloud,
           sl.MEASURE.XYZ
       )


       frame_bgra = zed_image.get_data()


       frame = cv2.cvtColor(
           frame_bgra,
           cv2.COLOR_BGRA2BGR
       )


       frame_h, frame_w = frame.shape[:2]


       (
           input_tensor,
           scale,
           pad_x,
           pad_y
       ) = preprocess(frame)


       output = infer(
           input_tensor
       )


       detections = decode(
           output,
           scale,
           pad_x,
           pad_y,
           frame_w,
           frame_h
       )


       # IMPORTANT:
       # This stays INSIDE while True
       for det in detections:

            u = int(
                (det["x1"] + det["x2"]) / 2
            )

            v = int(
                (det["y1"] + det["y2"]) / 2
            )

            xyz = get_xyz(
                point_cloud,
                u,
                v
            )

            if xyz is None:
                continue

            x, y, z = xyz

            intercept = updateProjectilePhysics(
                t,
                x,
                y,
                z
            )

            print(
                f"conf={det['confidence']:.2f} "
                f"XYZ=({x:.3f}, {y:.3f}, {z:.3f}) m"
            )

            if intercept is not None:

                point = intercept["point"]

                print(
                    f"INTERCEPT XYZ=("
                    f"{point[0]:.3f}, "
                    f"{point[1]:.3f}, "
                    f"{point[2]:.3f}) m"
                )

                print(
                    f"Intercept time="
                    f"{intercept['interceptTime']:.3f}s "
                    f"flight time="
                    f"{intercept['flightTime']:.3f}s"
                )

                print(
                    f"Aim yaw="
                    f"{intercept['yawDegrees']:.2f} deg "
                    f"pitch="
                    f"{intercept['pitchDegrees']:.2f} deg"
                )

finally:

    zed.close()
    cudaContext.pop()