//Captures a set amount of data from the camera

//Optitrack Libraries
#include <NatNetTypes.h>
#include <NatNetClient.h>

//GNU Libraries
#include <iostream>
#include <fstream>
#include <filesystem>
#include <unordered_map>
#include <chrono>
#include <thread>
#include <mutex>
#include <string>
using std::cout, std::cerr;

NatNetClient* global_client = nullptr;
sNatNetClientConnectParams global_connect_params;
sServerDescription global_server_description;
sDataDescriptions* global_data_defs = nullptr;
std::ofstream output_file;
std::unordered_map<int, uint8_t> bone_ID_to_first_column;
std::unordered_map<int, std::string> bone_ID_to_name;

std::mutex file_mutex;

void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData);


int main(int argc, char* argv[]){
    if(argc < 3 || argc > 4){
        cerr << "USAGE: \"" << argv[0] << "\": time_minutes output_file [time_seconds]\n"; 
        exit(1);
    }

    //parse time_minutes argument
    float recording_time;
    try{
        recording_time = std::stof(argv[1]);
    } catch (const std::exception) {
        cerr << argv[0] << ": Invalid time_minutes\n";
        exit(1);
    }

    std::filesystem::path output_file_path(argv[2]);
    { //parse output_file path
        //is the filename a directory?
        if(std::filesystem::exists(output_file_path) && std::filesystem::is_directory(output_file_path)){
            cerr << argv[0] << ": output_file cannot be a directory\n";
            exit(1);
        } 

        //does the parent path exist?
        std::filesystem::path parent_path = output_file_path.parent_path();
        //if path isn't relative and the parent path isn't valid
        if(!parent_path.empty() && (!std::filesystem::exists(parent_path) || !std::filesystem::is_directory(parent_path))){
            cerr << argv[0] << ": output directory \"" << parent_path << "\" is invalid\n";
            exit(1);
        }

        output_file.open(output_file_path);

        if(output_file.bad()){
        cerr << argv[0] << ": output_file could not be opened\n";
        }
    }

    //NatNet Initialization
    global_client = new NatNetClient();

    //connect to API endpoint
    global_connect_params.localAddress = "127.0.0.1"; //localhost
    global_connect_params.serverAddress = "127.0.0.1"; //localhost
    global_connect_params.connectionType = ConnectionType_Multicast; //faster
    ErrorCode ret = global_client->Connect(global_connect_params);
    if (ret != ErrorCode_OK) {
        cerr << "Failed to connect to Motive\n";
        return 1;
    }

    //get server description
    memset(&global_server_description, 0, sizeof(global_server_description));
    ret = global_client->GetServerDescription(&global_server_description);
    if (ret != ErrorCode_OK || !global_server_description.HostPresent)
    {
        printf("Unable to get server description. Error Code:%d.  Exiting.\n", ret);
        return 1;
    }
    else
    {
        printf("Connected : %s (ver. %d.%d.%d.%d)\n", global_server_description.szHostApp, global_server_description.HostAppVersion[0],
            global_server_description.HostAppVersion[1], global_server_description.HostAppVersion[2], global_server_description.HostAppVersion[3]);
    }

    //get data asset list 
    ret = global_client->GetDataDescriptionList(&global_data_defs, 0b0000100); //get skeleton description
    if (ret != ErrorCode_OK || global_data_defs == NULL)
    {
        printf("Error getting asset list.  Error Code:%d  Exiting.\n", ret);
        return 1;
    }

    //collect joint names from asset list (IDs may not be consistent between runs)
    {
        bool found_skeleton_data = false;
        for(unsigned int i = 0; i < global_data_defs->nDataDescriptions; i++){
            if(global_data_defs->arrDataDescriptions[i].type == Descriptor_Skeleton){
                if(found_skeleton_data == 1){
                    cerr << argv[0] << ": error, ensure only one skeleton model is selected in motive\n";
                    exit(1);
                }
                found_skeleton_data = 1;
                sSkeletonDescription* pSK = global_data_defs->arrDataDescriptions[i].Data.SkeletonDescription;
                for(int j = 0; j < pSK->nRigidBodies; j++){
                    sRigidBodyDescription* pRB = &pSK->RigidBodies[j];
                    bone_ID_to_name.emplace(pRB->ID, pRB->szName);
                }

                //send configuration to file
                output_file << "Motive Version:,\"" <<  global_server_description.HostAppVersion[1] << global_server_description.HostAppVersion[2] << global_server_description.HostAppVersion[3] << "\",";
                output_file << "Skeleton_Name:,\"" << pSK->szName << '\n';
            }
        }
    }

    cout << "Finished Motive Connection... Recording to \"" << std::filesystem::absolute(output_file_path) << "\"\n";

    //parse wait time
    if(argc == 4){
        try{
            float wait_time = std::stof(argv[3]);
            cout << "Waiting " << wait_time << "seconds...\n";
            std::this_thread::sleep_for(std::chrono::duration<float>(wait_time));
        } catch (const std::exception){
            cerr << argv[0] << ": Invalid wait_time\n";
        }
    }

    //use rigid body names to print CSV header

    //first line of header is joint name
    output_file << ","; //make space for timestamp and skeletonID
    {
        uint8_t current_column = 0;
        for(auto& pair : bone_ID_to_name){
            bone_ID_to_first_column.emplace(pair.first, current_column);
            current_column += 7;
            for(unsigned int i = 0; i < 7; i++){ //each rigid body has 6 parameters
                output_file << ',' << pair.second;
            }
        }
    }

    //second line is measurememnt
    output_file << "\ntimestamp,skeletonID,";
    for(unsigned int i = 0; i < bone_ID_to_name.size(); i++){
        output_file << "x,y,z,qw,qx,qy,qz";
    }
    output_file << '\n';
    
    //set the frame callback handler
    ret = global_client->SetFrameReceivedCallback(DataHandler, global_client);	

    std::this_thread::sleep_for(std::chrono::duration<float>(recording_time));
    cout << "Done!\n";

    global_client->Disconnect();
    delete global_client;
    exit(0);
}

void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData){
    if(output_file.bad()) exit(1);

    const uint64_t timestamp = data->CameraMidExposureTimestamp;
    std::lock_guard<std::mutex> lock(file_mutex);
    
    //print each skeleton's data
    const size_t column_count = bone_ID_to_first_column.size() * 7;
    static float* column_data = new float[column_count]; //initialize the memory once
    std::fill(column_data, column_data + column_count, std::numeric_limits<float>::quiet_NaN()); //intialize every column to NaN

    for(unsigned int i = 0; i < data->nSkeletons; i++){ //for each skeleton in data
        output_file << timestamp << ',' << data->Skeletons[i].skeletonID;
        for(unsigned int j = 0; j < data->Skeletons[i].nRigidBodies; j++){ //for each rigidbody in skeleton
            try{
                size_t start_column = bone_ID_to_first_column.at(data->Skeletons[i].RigidBodyData[j].ID);
                column_data[start_column + 0] = data->Skeletons[i].RigidBodyData[j].x;
                column_data[start_column + 1] = data->Skeletons[i].RigidBodyData[j].y;
                column_data[start_column + 2] = data->Skeletons[i].RigidBodyData[j].z;
                column_data[start_column + 3] = data->Skeletons[i].RigidBodyData[j].qw;
                column_data[start_column + 4] = data->Skeletons[i].RigidBodyData[j].qx;
                column_data[start_column + 5] = data->Skeletons[i].RigidBodyData[j].qy;
                column_data[start_column + 6] = data->Skeletons[i].RigidBodyData[j].qz;
            } catch (const std::exception){
                cerr << "warning, boneID not mapped to name\n";
            }
        }

        //send bones to file
        for(unsigned int j = 0; j < column_count; j++){
            output_file << ',' << column_data[j];
        }
        output_file << '\n';
    }
}