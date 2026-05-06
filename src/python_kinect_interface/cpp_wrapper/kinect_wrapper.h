#pragma once

#include <stdint.h> //I love well defined data-type

#ifdef _WIN32
#define DLLCALL __declspec(dllexport)
#else
#error "you must use a windows machine"
#define DLLCALL
#endif

extern "C" {

//define sensor constants
const uint32_t KINECT_JOINT_COUNT = 25;
const uint32_t KINECT_MAX_BODIES = 6;

//store joint coordinates
typedef struct coordinate_3dC {
    float x;
    float y;
    float z;
} coordinate_3dC;

//store joint angle
typedef struct quaternionC {
    float x;
    float y;
    float z;
    float w;
} quaternionC;

//represents all data about a single joint
typedef struct JointC {
    int32_t joint_type;
    int32_t tracking_state;
    coordinate_3dC position;
    quaternionC orientation;
} JointC;

//represents the joints in a given body
typedef struct BodyDataC {
    uint64_t body_id;
    JointC joints[KINECT_JOINT_COUNT];
    quaternionC orientations[KINECT_JOINT_COUNT];
    int64_t timestamp;
} BodyDataC;

//create the class and null initialize variables
DLLCALL void*   kinect_create();

//connect to the kinect sensor
DLLCALL long    kinect_initialize(void* kinect);

//disconnect from the kinect sensor
DLLCALL long    kinect_close(void* kinect);

//pause the current thread and wait for a frame
DLLCALL long    kinect_wait_for_frame(void* kinect, uint32_t timeout_ms);

//get the latest data snapshot
DLLCALL long    kinect_get_latest_bodies(void* kinect, BodyDataC* bodies, int max_bodies, int* out_count);

//teardown the class and free its memory
DLLCALL long    kinect_destroy(void* kinect);

}