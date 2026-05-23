Requires the Optitrack NetNat SDK installed on your device.
To build, run: 

cmake -DNATNET_ROOT="C:/Path/To/NatNetSDK" -S . -B build
cmake --build build

Command line options:

recorder time_minutes output_file [wait_time_seconds]

- `time_minutes`: how long to record data before exiting.
- `output_file`: path to the CSV file to write.
- `time_seconds`: optional delay before recording starts.