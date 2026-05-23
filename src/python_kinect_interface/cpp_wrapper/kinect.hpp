#pragma once

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
#include <cmath>
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
    std::array<ColorSpacePoint, JointType_Count> color_points{};
};

class kinect {
private:
    WAITABLE_HANDLE event_notifier = NULL;
    ComPtr<IKinectSensor> kinect_sensor;
    ComPtr<ICoordinateMapper> coordinate_mapper;
    ComPtr<IBodyFrameReader> body_reader;
    ComPtr<IColorFrameReader> color_reader;
    int color_width = 1920;
    int color_height = 1080;
    int color_bytes_per_pixel = 4;

    void cleanup_reader_subscription() noexcept {
        if (body_reader && event_notifier) {
            body_reader->UnsubscribeFrameArrived(event_notifier);
            event_notifier = NULL;
        }
    }

    static void set_invalid_color_points(std::array<ColorSpacePoint, JointType_Count>& points) noexcept {
        for (auto& point : points) {
            point.X = -1.0f;
            point.Y = -1.0f;
        }
    }

public:
    kinect() = default;

    kinect(const kinect&) = delete;
    kinect& operator=(const kinect&) = delete;

    kinect(kinect&&) = delete;
    kinect& operator=(kinect&&) = delete;

    HRESULT initialize() {
        cleanup_reader_subscription();
        body_reader.Reset();
        color_reader.Reset();
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

        ComPtr<IBodyFrameSource> body_source;
        hr = kinect_sensor->get_BodyFrameSource(body_source.GetAddressOf());
        if (FAILED(hr)) {
            cerr << "Failed to initialize body frame source. Kinect not ready...\n";
            return hr;
        }

        hr = body_source->OpenReader(body_reader.GetAddressOf());
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

        ComPtr<IColorFrameSource> color_source;
        hr = kinect_sensor->get_ColorFrameSource(color_source.GetAddressOf());
        if (FAILED(hr)) {
            cerr << "Failed to initialize color frame source. Kinect not ready...\n";
            cleanup_reader_subscription();
            body_reader.Reset();
            return hr;
        }

        ComPtr<IFrameDescription> color_description;
        hr = color_source->get_FrameDescription(color_description.GetAddressOf());
        if (SUCCEEDED(hr) && color_description) {
            color_description->get_Width(&color_width);
            color_description->get_Height(&color_height);
        }
        color_bytes_per_pixel = 4;

        hr = color_source->OpenReader(color_reader.GetAddressOf());
        if (FAILED(hr)) {
            cerr << "Failed to initialize color frame reader. Kinect not ready...\n";
            cleanup_reader_subscription();
            body_reader.Reset();
            return hr;
        }

        return S_OK;
    }

    std::pair<WAITABLE_HANDLE, IBodyFrameReader*> get_raw_notifier() const {
        return {event_notifier, body_reader.Get()};
    }

    ~kinect() {
        cleanup_reader_subscription();

        color_reader.Reset();
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
            set_invalid_color_points(data.color_points);

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

            if (coordinate_mapper) {
                for (size_t i = 0; i < JointType_Count; ++i) {
                    if (data.joints[i].TrackingState == TrackingState_NotTracked) {
                        continue;
                    }

                    ColorSpacePoint color_point{};
                    HRESULT map_hr = coordinate_mapper->MapCameraPointToColorSpace(
                        data.joints[i].Position,
                        &color_point
                    );

                    if (SUCCEEDED(map_hr) && std::isfinite(color_point.X) && std::isfinite(color_point.Y)) {
                        data.color_points[i] = color_point;
                    }
                }
            }

            out.emplace_back(std::move(data));
        }

        return {time, std::move(out)};
    }

    HRESULT wait_for_next_frame(uint32_t timeout) {
        if (!event_notifier) {
            return E_INVALIDARG;
        }

        DWORD wait_result = WaitForSingleObject(reinterpret_cast<HANDLE>(event_notifier), timeout);

        if (wait_result == WAIT_FAILED) {
            return HRESULT_FROM_WIN32(GetLastError());
        }

        return static_cast<HRESULT>(wait_result);
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

    HRESULT get_color_frame_size(int* out_width, int* out_height, int* out_bytes_per_pixel) const {
        if (!out_width || !out_height || !out_bytes_per_pixel) {
            return E_INVALIDARG;
        }

        *out_width = color_width;
        *out_height = color_height;
        *out_bytes_per_pixel = color_bytes_per_pixel;
        return S_OK;
    }

    HRESULT get_latest_color_frame_bgra(
        uint8_t* buffer,
        int buffer_size,
        int* out_width,
        int* out_height,
        TIMESPAN* out_timestamp
    ) {
        if (!color_reader || !buffer || !out_width || !out_height || !out_timestamp) {
            return E_INVALIDARG;
        }

        const int required_size = color_width * color_height * color_bytes_per_pixel;
        if (buffer_size < required_size) {
            return HRESULT_FROM_WIN32(ERROR_INSUFFICIENT_BUFFER);
        }

        ComPtr<IColorFrame> color_frame;
        HRESULT hr = color_reader->AcquireLatestFrame(color_frame.GetAddressOf());
        if (FAILED(hr)) {
            return hr;
        }

        hr = color_frame->get_RelativeTime(out_timestamp);
        if (FAILED(hr)) {
            return hr;
        }

        hr = color_frame->CopyConvertedFrameDataToArray(
            static_cast<UINT>(required_size),
            buffer,
            ColorImageFormat_Bgra
        );
        if (FAILED(hr)) {
            return hr;
        }

        *out_width = color_width;
        *out_height = color_height;
        return S_OK;
    }

    void deinnit() {
        cleanup_reader_subscription();

        color_reader.Reset();
        body_reader.Reset();
        coordinate_mapper.Reset();

        if (kinect_sensor) {
            kinect_sensor->Close();
            kinect_sensor.Reset();
        }
    }
};
