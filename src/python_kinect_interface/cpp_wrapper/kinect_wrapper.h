#pragma once

#include <stdint.h>

#ifdef _WIN32
#define DLLCALL __declspec(dllexport)
#else
#error "you must use a windows machine"
#define DLLCALL
#endif

extern "C" {

//sensor constants for Kinect v2.
const uint32_t KINECT_JOINT_COUNT = 25;
const uint32_t KINECT_MAX_BODIES = 6;
const uint32_t KINECT_COLOR_WIDTH = 1920;
const uint32_t KINECT_COLOR_HEIGHT = 1080;
const uint32_t KINECT_COLOR_BYTES_PER_PIXEL = 4; // BGRA/ARGB32-compatible bytes.

//store joint coordinates in meters.
typedef struct coordinate_3dC {
    float x;
    float y;
    float z;
} coordinate_3dC;

//store a projected 2-D point in Kinect color-image pixel coordinates.
typedef struct coordinate_2dC {
    float x;
    float y;
} coordinate_2dC;

//store joint orientation.
typedef struct quaternionC {
    float x;
    float y;
    float z;
    float w;
} quaternionC;

//represents all data about a single joint.
typedef struct JointC {
    int32_t joint_type;
    int32_t tracking_state;
    coordinate_3dC position;
    quaternionC orientation;
    coordinate_2dC color_position;
} JointC;

//represents the joints in a given body.
typedef struct BodyDataC {
    uint64_t body_id;
    JointC joints[KINECT_JOINT_COUNT];
    quaternionC orientations[KINECT_JOINT_COUNT];
    int64_t timestamp;
} BodyDataC;

//create the class and null initialize variables.
DLLCALL void* kinect_create();

//connect to the Kinect sensor.
DLLCALL long kinect_initialize(void* kinect);

//disconnect from the Kinect sensor.
DLLCALL long kinect_close(void* kinect);

//pause the current thread and wait for a body frame event.
// --returns WAIT_OBJECT_0 (0) on success, WAIT_TIMEOUT (258) on timeout,
// --or an HRESULT/Win32 error on failure.
DLLCALL long kinect_wait_for_frame(void* kinect, uint32_t timeout_ms);

//get the latest raw body data snapshot.
DLLCALL long kinect_get_latest_bodies(void* kinect, BodyDataC* bodies, int max_bodies, int* out_count);

//get the latest normalized body data snapshot. Positions are translated to
// SpineBase and rotated so SpineBase orientation matches TARGET_BASE_QUATERNION.
DLLCALL long kinect_get_latest_bodies_normalized(void* kinect, BodyDataC* bodies, int max_bodies, int* out_count);

//get raw and normalized body arrays from the same acquired Kinect body frame.
DLLCALL long kinect_get_latest_body_frame(
    void* kinect,
    BodyDataC* raw_bodies,
    BodyDataC* normalized_bodies,
    int max_bodies,
    int* out_count
);

//query the color frame dimensions used by kinect_get_latest_color_frame.
DLLCALL long kinect_get_color_frame_size(
    void* kinect,
    int* out_width,
    int* out_height,
    int* out_bytes_per_pixel
);

//copy the latest Kinect color frame into bgra_buffer. Buffer size must be at
// least width * height * 4 bytes. Pixel bytes are BGRA, compatible with a little-endian Qt ARGB32 QImage.
DLLCALL long kinect_get_latest_color_frame(
    void* kinect,
    uint8_t* bgra_buffer,
    int buffer_size,
    int* out_width,
    int* out_height,
    int64_t* out_timestamp
);

//teardown the class and free its memory.
DLLCALL long kinect_destroy(void* kinect);

}
