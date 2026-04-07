//Captures a set amount of data from the camera and saves it to a csv file

//Windows headers
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <windows.h>
#include <Shlobj.h>

//Direct2D Header
#include <d2d1.h>

//Kinect Header
#include <Kinect.h>

//std headers
#include <iostream>
using std::cout, std::cerr;


class reader {
public:
    reader(): 
        kinect_sensor(nullptr),
        coordinate_mapper(nullptr),
        body_reader(nullptr),
        event_notifier(0)
    {};

    HRESULT initialize(){
        HRESULT result;

        //initialize the usb connection
        if (kinect_sensor){
            kinect_sensor->Close();
            kinect_sensor->Release();
            kinect_sensor = nullptr;
        }
        
        result = GetDefaultKinectSensor(&kinect_sensor);
        if(FAILED(result)) return result;
        if(kinect_sensor == nullptr){
            cerr << "No kinect found...\n";
            return E_FAIL;
        }

        result = kinect_sensor->Open();
        if(FAILED(result)) {
            cerr << "Failed to start interface. Kinect not ready...\n";
            return result;
        }

        //initialize the coordinate mapper
        if(coordinate_mapper){
            coordinate_mapper->Release();
            coordinate_mapper = nullptr;
        }

        result = kinect_sensor->get_CoordinateMapper(&coordinate_mapper);
        if(FAILED(result)) {
            cerr << "Failed to get coordinate mapper. Kinect not ready...\n";
            return result;
        }

        //initialize the body frame source
        IBodyFrameSource* fs = nullptr;
        result = kinect_sensor->get_BodyFrameSource(&fs);
        if(FAILED(result)){
            cerr << "Failed to initialize body frame source. Kinect not ready...\n";
            if(fs) fs->Release();
            return result;
        }

        //initialize the body frame reader
        if(body_reader){
            body_reader->Release();
            body_reader = nullptr;
        }

        result = fs->OpenReader(&body_reader);
        if(FAILED(result)) {
            cerr << "Failed to initialize body frame reader. Kinect not ready...\n";
            if(fs) fs->Release();
            return result;
        }
        if(fs) fs->Release();

        //create the notification event
        result = body_reader->SubscribeFrameArrived(&event_notifier);

        return result;
    }

    std::pair<WAITABLE_HANDLE, IBodyFrameReader*>  get_notifier(){
        return std::make_pair(event_notifier, body_reader);
    }

    ~reader(){
        //done with body frame reader
        if(body_reader){
            body_reader->Release();
            body_reader = nullptr;
        }

        //done with coordinate mapper
        if(coordinate_mapper){
            coordinate_mapper->Release();
            coordinate_mapper = nullptr;
        }

        //done with sensor
        if (kinect_sensor){
            kinect_sensor->Close();
            kinect_sensor->Release();
            kinect_sensor = nullptr;
        }
    }
    
private:
    WAITABLE_HANDLE     event_notifier;
    IKinectSensor*      kinect_sensor;
    ICoordinateMapper*  coordinate_mapper;
    IBodyFrameReader*   body_reader;
};