Requires the Microsoft Kinect for Windows SDK 2.0 installed on your device.

To build, run:

cmake -DKINECTSDK20_ROOT="C:/Program Files/Microsoft SDKs/Kinect/v2.0_1409" -S . -B build
cmake --build build

Command line options:

writer time_minutes output_file [wait_time_seconds]

- `time_minutes`: how long to record data before exiting.
- `output_file`: path to the CSV file to write.
- `time_seconds`: optional delay before recording starts.
