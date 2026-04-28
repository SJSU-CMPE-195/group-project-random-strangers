#include "kinect_wrapper.h"
#include "kinect.hpp"

#include <new>

extern "C"{

DLLCALL void* kinect_create(){
    //create an object but don't crash everything if it doesn't work
    try { 
        return new kinect();
    } catch (...) {
        return nullptr; 
    }
}

DLLCALL long kinect_initialize(void* device){
    if(device == NULL) return E_INVALIDARG;

    kinect* kinect_class = static_cast<kinect*>(device);

    HRESULT return_code = kinect_class->initialize();

    return return_code;
}

DLLCALL long kinect_close(void* device){
    if(device == NULL) return E_INVALIDARG;

    kinect* kinect_class = static_cast<kinect*>(device);

    kinect_class->deinnit();

    return S_OK;
}

DLLCALL long kinect_wait_for_frame(void* device, uint32_t timeout_ms){
    if(device == NULL) return E_INVALIDARG;

    kinect* kinect_class = static_cast<kinect*>(device);

    HRESULT return_code = kinect_class->wait_for_next_frame(timeout_ms);

    return return_code;
}

DLLCALL long kinect_get_latest_bodies(void* device, BodyDataC* bodies, int max_bodies, int* out_count){
    if(device == NULL || bodies == NULL || out_count == NULL) return E_INVALIDARG;

    kinect* kinect_class = static_cast<kinect*>(device);

    auto [timestamp, body_data] = kinect_class->get_latest_joint_data();

    if (timestamp < 0) {
        return timestamp; //error code
    }

    int count = 0;
    for (const auto& body : body_data) {
        if (count >= max_bodies) {
            break;
        }

        bodies[count].body_id = body.body_id;
        bodies[count].timestamp = timestamp;

        for (size_t i = 0; i < KINECT_JOINT_COUNT; ++i) {
            const auto& joint = body.joints[i];            

            bodies[count].joints[i].joint_type = joint.JointType;
            bodies[count].joints[i].tracking_state = joint.TrackingState;

            bodies[count].joints[i].position.x = joint.Position.X;
            bodies[count].joints[i].position.y = joint.Position.Y;
            bodies[count].joints[i].position.z = joint.Position.Z;

            const auto& orientation = body.orientations[i];
            bodies[count].joints[i].orientation.x = orientation.Orientation.x;
            bodies[count].joints[i].orientation.y = orientation.Orientation.y;
            bodies[count].joints[i].orientation.z = orientation.Orientation.z;
            bodies[count].joints[i].orientation.w = orientation.Orientation.z;
        }

        ++count;
    }

    *out_count = count;

    return S_OK;
}

DLLCALL long kinect_destroy(void* device){
    if(device == NULL) return E_INVALIDARG;

    kinect* kinect_class = static_cast<kinect*>(device);

    delete kinect_class;

    return S_OK;
}

}