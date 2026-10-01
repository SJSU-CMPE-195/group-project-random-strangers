import cv2
import numpy as np
import tensorrt as trt
import pycuda.autoinit
import pycuda.driver as imgoinginsane
import pyzed.sl as sl

# engine_path = ~/documents/group_random_strangers/software/best.engine  #path to engine file
confidenceIndex = 0.25 
targetIndex = 1 # maybe 2, if we want to attack people lmao
#IOUthreshold -> intersection over union, maybe worth looking at for precision 

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


def infer(input_tensor):

    # Copy processed image into CPU input buffer
    np.copyto(
        host_input,
        input_tensor
    )

    # Copy CPU input → GPU input
    cuda.memcpy_htod_async(
        device_input,
        host_input,
        stream
    )

    # Run TensorRT
    context.execute_async_v3(
        stream_handle=stream.handle
    )

    # Copy GPU output toCPU output
    cuda.memcpy_dtoh_async(
        host_output,
        device_output,
        stream
    )

    # Wait for GPU operations to finish
    stream.synchronize()

    # Return independent copy of results
    return host_output.copy()


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


    for pred in predictions:

        # First four numbers:
        #
        # center X
        # center Y
        # width
        # height
        cx, cy, w, h = pred[:4]


        # Remaining numbers are class scores
        class_scores = pred[4:]


        # Find class with highest score
        class_id = int(
            np.argmax(class_scores)
        )

        confidence = float(
            class_scores[class_id]
        )


        # Ignore weak detections
        if confidence < confidenceIndex:
            continue


        # Ignore detections that aren't the target class
        if class_id != targetIndex:
            continue


        # Convert:
        #
        # center + width/height
        #
        # into:
        #
        # corner coordinates
        x1 = cx - w / 2
        y1 = cy - h / 2

        x2 = cx + w / 2
        y2 = cy + h / 2


        # Undo the letterboxing
        x1 = (
            x1 - pad_x
        ) / scale

        y1 = (
            y1 - pad_y
        ) / scale

        x2 = (
            x2 - pad_x
        ) / scale

        y2 = (
            y2 - pad_y
        ) / scale


        # Keep coordinates inside image bounds
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


        # OpenCV NMSBoxes expect
    
        # x
        # y
        # width
        # height
        boxes.append([
            int(x1),
            int(y1),
            int(x2 - x1),
            int(y2 - y1)
        ])

        scores.append(
            confidence
        )

        class_ids.append(
            class_id
        )


    # Nothing detected
    if not boxes:
        return []


    # Remove overlapping duplicate detections
    indices = cv2.dnn.NMSBoxes(
        boxes,
        scores,
        confidenceIndex,
        #IOUIndex
    )


    detections = []


    for i in indices:

        i = int(i)

        x, y, w, h = boxes[i]

        detections.append({
            "x1": x,
            "y1": y,
            "x2": x + w,
            "y2": y + h,
            "confidence": scores[i],
            "class_id": class_ids[i]
        })


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


# ============================================================
# CREATE AND CONFIGURE ZED CAMERA
# ============================================================

zed = sl.Camera()

init = sl.InitParameters()


# Camera mode
init.camera_resolution = (
    sl.RESOLUTION.HD720
)

init.camera_fps = 60


# Fast depth mode
init.depth_mode = (
    sl.DEPTH_MODE.PERFORMANCE
)


# XYZ values will be meters
init.coordinate_units = (
    sl.UNIT.METER
)


# Make positive Y point upward
init.coordinate_system = (
    sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
)


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

        # ----------------------------------------------------
        # 1. Grab new stereo frame
        # ----------------------------------------------------

        if (
            zed.grab(runtime_params)
            != sl.ERROR_CODE.SUCCESS
        ):
            continue


        # ----------------------------------------------------
        # 2. Retrieve left camera image
        # ----------------------------------------------------

        zed.retrieve_image(
            zed_image,
            sl.VIEW.LEFT
        )


        # ----------------------------------------------------
        # 3. Retrieve XYZ point cloud
        # ----------------------------------------------------

        zed.retrieve_measure(
            point_cloud,
            sl.MEASURE.XYZ
        )


        # ----------------------------------------------------
        # 4. Convert ZED image into OpenCV / NumPy array
        # ----------------------------------------------------

        frame_bgra = (
            zed_image.get_data()
        )


        frame = cv2.cvtColor(
            frame_bgra,
            cv2.COLOR_BGRA2BGR
        )


        frame_h, frame_w = (
            frame.shape[:2]
        )


        # ----------------------------------------------------
        # 5. Preprocess image
        # ----------------------------------------------------

        (
            input_tensor,
            scale,
            pad_x,
            pad_y
        ) = preprocess(frame)


        # ----------------------------------------------------
        # 6. Run YOLO through TensorRT
        # ----------------------------------------------------

        output = infer(
            input_tensor
        )


        # ----------------------------------------------------
        # 7. Turn raw YOLO output into detections
        # ----------------------------------------------------

        detections = decode(
            output,
            scale,
            pad_x,
            pad_y,
            frame_w,
            frame_h
        )


        # ----------------------------------------------------
        # 8. Process each detected ball
        # ----------------------------------------------------

        for det in detections:

            # Find center of bounding box
            u = int(
                (
                    det["x1"]
                    + det["x2"]
                ) / 2
            )

            v = int(
                (
                    det["y1"]
                    + det["y2"]
                ) / 2
            )


            # ------------------------------------------------
            # 9. Ask ZED for XYZ at that location
            # ------------------------------------------------

            xyz = get_xyz(
                point_cloud,
                u,
                v
            )


            if xyz is None:
                continue


            x, y, z = xyz


            # ------------------------------------------------
            # 10. Print projectile position
            # ------------------------------------------------

            print(
                f"conf={det['confidence']:.2f} "
                f"XYZ=("
                f"{x:.3f}, "
                f"{y:.3f}, "
                f"{z:.3f}"
                f") m"
            )

finally:

    zed.close()