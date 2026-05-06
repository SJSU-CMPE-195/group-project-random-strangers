#include "kinect_wrapper.h"
#include "kinect.hpp"
#include "rot_pos_normalizer.hpp"

#include <cmath>
#include <new>

const quaternionC TARGET_BASE_QUATERNION {0.0f, 1.0f, 0.0f, 0.0f};

namespace {

coordinate_3dC to_coordinate_3d(const CameraSpacePoint& point) {
    return coordinate_3dC {point.X, point.Y, point.Z};
}

coordinate_2dC to_coordinate_2d(const ColorSpacePoint& point) {
    if (!std::isfinite(point.X) || !std::isfinite(point.Y)) {
        return coordinate_2dC {-1.0f, -1.0f};
    }
    return coordinate_2dC {point.X, point.Y};
}

quaternionC to_quaternion(const JointOrientation& orientation) {
    return quaternionC {
        orientation.Orientation.x,
        orientation.Orientation.y,
        orientation.Orientation.z,
        orientation.Orientation.w
    };
}

bool valid_quaternion(const quaternionC& q) {
    const float norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return std::isfinite(norm2) && norm2 > 0.0001f;
}

JointC make_raw_joint(const Body_Data_t& body, size_t index) {
    JointC out{};
    const auto& joint = body.joints[index];

    out.joint_type = joint.JointType;
    out.tracking_state = joint.TrackingState;
    out.color_position = to_coordinate_2d(body.color_points[index]);

    if (joint.TrackingState == TrackingState_NotTracked) {
        out.position = coordinate_3dC {0.0f, 0.0f, 0.0f};
        out.orientation = quaternionC {0.0f, 0.0f, 0.0f, 0.0f};
        return out;
    }

    out.position = to_coordinate_3d(joint.Position);
    out.orientation = to_quaternion(body.orientations[index]);
    return out;
}

void copy_body_raw(BodyDataC& out, const Body_Data_t& body, TIMESPAN timestamp) {
    out = BodyDataC{};
    out.body_id = body.body_id;
    out.timestamp = timestamp;

    for (size_t i = 0; i < KINECT_JOINT_COUNT; ++i) {
        out.joints[i] = make_raw_joint(body, i);
        out.orientations[i] = out.joints[i].orientation;
    }
}

void copy_body_normalized(BodyDataC& out, const Body_Data_t& body, TIMESPAN timestamp) {
    out = BodyDataC{};
    out.body_id = body.body_id;
    out.timestamp = timestamp;

    const auto& base_joint = body.joints[JointType_SpineBase];
    const coordinate_3dC translation {
        -base_joint.Position.X,
        -base_joint.Position.Y,
        -base_joint.Position.Z
    };

    const quaternionC base_orientation = to_quaternion(body.orientations[JointType_SpineBase]);
    const bool can_rotate = base_joint.TrackingState != TrackingState_NotTracked && valid_quaternion(base_orientation);
    const quaternionC required_rotation = can_rotate
        ? get_rotator(base_orientation, TARGET_BASE_QUATERNION)
        : quaternionC {0.0f, 0.0f, 0.0f, 1.0f};

    for (size_t i = 0; i < KINECT_JOINT_COUNT; ++i) {
        JointC joint = make_raw_joint(body, i);

        if (joint.tracking_state != TrackingState_NotTracked) {
            joint.position = joint.position + translation;
            joint.position = rotate_point(joint.position, required_rotation);

            if (valid_quaternion(joint.orientation)) {
                joint.orientation = rotate_quaternion_global(joint.orientation, required_rotation);
            }
        }

        if (i == JointType_SpineBase && joint.tracking_state != TrackingState_NotTracked) {
            joint.position = coordinate_3dC {0.0f, 0.0f, 0.0f};
            joint.orientation = can_rotate ? TARGET_BASE_QUATERNION : joint.orientation;
        }

        out.joints[i] = joint;
        out.orientations[i] = joint.orientation;
    }
}

long fill_body_arrays(
    const std::vector<Body_Data_t>& body_data,
    TIMESPAN timestamp,
    BodyDataC* raw_bodies,
    BodyDataC* normalized_bodies,
    int max_bodies,
    int* out_count
) {
    if (max_bodies < 0 || out_count == nullptr) {
        return E_INVALIDARG;
    }

    int count = 0;
    for (const auto& body : body_data) {
        if (count >= max_bodies) {
            break;
        }

        if (raw_bodies) {
            copy_body_raw(raw_bodies[count], body, timestamp);
        }

        if (normalized_bodies) {
            copy_body_normalized(normalized_bodies[count], body, timestamp);
        }

        ++count;
    }

    *out_count = count;
    return S_OK;
}

} // namespace

extern "C" {

DLLCALL void* kinect_create() {
    try {
        return new kinect();
    } catch (...) {
        return nullptr;
    }
}

DLLCALL long kinect_initialize(void* device) {
    if (device == NULL) {
        return E_INVALIDARG;
    }

    kinect* kinect_class = static_cast<kinect*>(device);
    HRESULT return_code = kinect_class->initialize();
    return return_code;
}

DLLCALL long kinect_close(void* device) {
    if (device == NULL) {
        return E_INVALIDARG;
    }

    kinect* kinect_class = static_cast<kinect*>(device);
    kinect_class->deinnit();
    return S_OK;
}

DLLCALL long kinect_wait_for_frame(void* device, uint32_t timeout_ms) {
    if (device == NULL) {
        return E_INVALIDARG;
    }

    kinect* kinect_class = static_cast<kinect*>(device);
    HRESULT return_code = kinect_class->wait_for_next_frame(timeout_ms);
    return return_code;
}

DLLCALL long kinect_get_latest_bodies(void* device, BodyDataC* bodies, int max_bodies, int* out_count) {
    if (device == NULL || bodies == NULL || out_count == NULL) {
        return E_INVALIDARG;
    }

    kinect* kinect_class = static_cast<kinect*>(device);
    const auto latest = kinect_class->get_latest_joint_data();
    const TIMESPAN timestamp = latest.first;
    const std::vector<Body_Data_t>& body_data = latest.second;

    if (timestamp < 0) {
        return static_cast<long>(timestamp);
    }

    return fill_body_arrays(body_data, timestamp, bodies, nullptr, max_bodies, out_count);
}

DLLCALL long kinect_get_latest_bodies_normalized(void* device, BodyDataC* bodies, int max_bodies, int* out_count) {
    if (device == NULL || bodies == NULL || out_count == NULL) {
        return E_INVALIDARG;
    }

    kinect* kinect_class = static_cast<kinect*>(device);
    const auto latest = kinect_class->get_latest_joint_data();
    const TIMESPAN timestamp = latest.first;
    const std::vector<Body_Data_t>& body_data = latest.second;

    if (timestamp < 0) {
        return static_cast<long>(timestamp);
    }

    return fill_body_arrays(body_data, timestamp, nullptr, bodies, max_bodies, out_count);
}

DLLCALL long kinect_get_latest_body_frame(
    void* device,
    BodyDataC* raw_bodies,
    BodyDataC* normalized_bodies,
    int max_bodies,
    int* out_count
) {
    if (device == NULL || raw_bodies == NULL || normalized_bodies == NULL || out_count == NULL) {
        return E_INVALIDARG;
    }

    kinect* kinect_class = static_cast<kinect*>(device);
    const auto latest = kinect_class->get_latest_joint_data();
    const TIMESPAN timestamp = latest.first;
    const std::vector<Body_Data_t>& body_data = latest.second;

    if (timestamp < 0) {
        return static_cast<long>(timestamp);
    }

    return fill_body_arrays(body_data, timestamp, raw_bodies, normalized_bodies, max_bodies, out_count);
}

DLLCALL long kinect_get_color_frame_size(
    void* device,
    int* out_width,
    int* out_height,
    int* out_bytes_per_pixel
) {
    if (device == NULL) {
        return E_INVALIDARG;
    }

    kinect* kinect_class = static_cast<kinect*>(device);
    return kinect_class->get_color_frame_size(out_width, out_height, out_bytes_per_pixel);
}

DLLCALL long kinect_get_latest_color_frame(
    void* device,
    uint8_t* bgra_buffer,
    int buffer_size,
    int* out_width,
    int* out_height,
    int64_t* out_timestamp
) {
    if (device == NULL) {
        return E_INVALIDARG;
    }

    kinect* kinect_class = static_cast<kinect*>(device);
    return kinect_class->get_latest_color_frame_bgra(
        bgra_buffer,
        buffer_size,
        out_width,
        out_height,
        out_timestamp
    );
}

DLLCALL long kinect_destroy(void* device) {
    if (device == NULL) {
        return E_INVALIDARG;
    }

    kinect* kinect_class = static_cast<kinect*>(device);
    delete kinect_class;
    return S_OK;
}

}
