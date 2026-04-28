// Windows headers
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <windows.h>
#include <ShlObj.h>

// Kinect header
#include <Kinect.h>

// WRL ComPtr
#include <wrl/client.h>

// std headers
#include <array>
#include <cstdint>
#include <iostream>
#include <utility>
#include <vector>

using Microsoft::WRL::ComPtr;
using std::cerr;

struct Body_Data_t {
    uint64_t body_id = 0;
    std::array<Joint, JointType_Count> joints{};
    std::array<JointOrientation, JointType_Count> orientations{};
};

class device {
private:
    WAITABLE_HANDLE event_notifier = NULL;
    ComPtr<IKinectSensor> kinect_sensor;
    ComPtr<ICoordinateMapper> coordinate_mapper;
    ComPtr<IBodyFrameReader> body_reader;

    void cleanup_reader_subscription() noexcept {
        if (body_reader && event_notifier) {
            body_reader->UnsubscribeFrameArrived(event_notifier);
            event_notifier = NULL;
        }
    }

public:
    device() = default;

    device(const device&) = delete;
    device& operator=(const device&) = delete;

    device(device&&) = delete;
    device& operator=(device&&) = delete;

    HRESULT initialize() {
        cleanup_reader_subscription();
        body_reader.Reset();
        coordinate_mapper.Reset();

        if (kinect_sensor) {
            kinect_sensor->Close();
            kinect_sensor.Reset();
        }

        HRESULT hr = GetDefaultKinectSensor(kinect_sensor.GetAddressOf());
        if (FAILED(hr)) {
            return hr;
        }

        if (!kinect_sensor) {
            cerr << "No Kinect found...\n";
            return E_FAIL;
        }

        hr = kinect_sensor->Open();
        if (FAILED(hr)) {
            cerr << "Failed to start interface. Kinect not ready...\n";
            return hr;
        }

        hr = kinect_sensor->get_CoordinateMapper(coordinate_mapper.GetAddressOf());
        if (FAILED(hr)) {
            cerr << "Failed to get coordinate mapper. Kinect not ready...\n";
            return hr;
        }

        ComPtr<IBodyFrameSource> frame_source;
        hr = kinect_sensor->get_BodyFrameSource(frame_source.GetAddressOf());
        if (FAILED(hr)) {
            cerr << "Failed to initialize body frame source. Kinect not ready...\n";
            return hr;
        }

        hr = frame_source->OpenReader(body_reader.GetAddressOf());
        if (FAILED(hr)) {
            cerr << "Failed to initialize body frame reader. Kinect not ready...\n";
            return hr;
        }

        hr = body_reader->SubscribeFrameArrived(&event_notifier);
        if (FAILED(hr)) {
            cerr << "Failed to subscribe to body frame notifications.\n";
            body_reader.Reset();
            return hr;
        }

        return S_OK;
    }

    std::pair<WAITABLE_HANDLE, IBodyFrameReader*> get_raw_notifier() const {
        return {event_notifier, body_reader.Get()};
    }

    ~device() {
        cleanup_reader_subscription();

        body_reader.Reset();
        coordinate_mapper.Reset();

        if (kinect_sensor) {
            kinect_sensor->Close();
            kinect_sensor.Reset();
        }
    }

    std::pair<TIMESPAN, std::vector<Body_Data_t>> get_latest_joint_data() {
        if (!body_reader) {
            return {-1, {}};
        }

        TIMESPAN time = -1;
        HRESULT hr = S_OK;

        ComPtr<IBodyFrame> body_frame;
        hr = body_reader->AcquireLatestFrame(body_frame.GetAddressOf());
        if (FAILED(hr)) {
            return {-2, {}};
        }

        hr = body_frame->get_RelativeTime(&time);
        if (FAILED(hr)) {
            return {-3, {}};
        }

        std::array<ComPtr<IBody>, BODY_COUNT> bodies;
        IBody* raw_bodies[BODY_COUNT] = {};

        hr = body_frame->GetAndRefreshBodyData(BODY_COUNT, raw_bodies);
        if (FAILED(hr)) {
            return {-4, {}};
        }

        for (size_t i = 0; i < BODY_COUNT; ++i) {
            bodies[i].Attach(raw_bodies[i]);
        }

        std::vector<Body_Data_t> out;
        out.reserve(BODY_COUNT);

        for (const auto& body : bodies) {
            if (!body) {
                continue;
            }

            BOOLEAN tracked = FALSE;
            hr = body->get_IsTracked(&tracked);
            if (FAILED(hr) || !tracked) {
                continue;
            }

            Body_Data_t data{};

            hr = body->get_TrackingId(&data.body_id);
            if (FAILED(hr)) {
                continue;
            }

            hr = body->GetJoints(JointType_Count, data.joints.data());
            if (FAILED(hr)) {
                continue;
            }

            hr = body->GetJointOrientations(JointType_Count, data.orientations.data());
            if (FAILED(hr)) {
                continue;
            }

            out.emplace_back(std::move(data));
        }

        return {time, std::move(out)};
    }

    std::pair<TIMESPAN, std::vector<Body_Data_t>> get_next_joint_data() {
        if (!event_notifier) {
            return {-6, {}};
        }

        DWORD wait_result = WaitForSingleObject(reinterpret_cast<HANDLE>(event_notifier), 100);

        if (wait_result != WAIT_OBJECT_0) {
            return {-5, {}};
        }

        return get_latest_joint_data();
    }
};