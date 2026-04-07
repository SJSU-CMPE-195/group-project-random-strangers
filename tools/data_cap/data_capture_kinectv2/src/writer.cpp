//Captures a set amount of Kinect skeleton data and saves it to a csv file

//Kinect reader implementation
#include "kinect_reader.cpp"

//std headers
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <stdio.h>
#include <vector>
#include <string>

using std::cerr, std::cout;

struct bone_details {
	int ID;
	std::string name;
	int parent_ID;
	uint8_t first_column;
	bone_details(const int t_ID, std::string t_name, const int t_parent_ID)
		: ID(t_ID), name(std::move(t_name)), parent_ID(t_parent_ID), first_column(0) {}
};

std::vector<bone_details> get_bone_layout() {
	return {
		{JointType_SpineBase, "SpineBase", -1},
		{JointType_SpineMid, "SpineMid", JointType_SpineBase},
		{JointType_Neck, "Neck", JointType_SpineShoulder},
		{JointType_Head, "Head", JointType_Neck},
		{JointType_ShoulderLeft, "ShoulderLeft", JointType_SpineShoulder},
		{JointType_ElbowLeft, "ElbowLeft", JointType_ShoulderLeft},
		{JointType_WristLeft, "WristLeft", JointType_ElbowLeft},
		{JointType_HandLeft, "HandLeft", JointType_WristLeft},
		{JointType_ShoulderRight, "ShoulderRight", JointType_SpineShoulder},
		{JointType_ElbowRight, "ElbowRight", JointType_ShoulderRight},
		{JointType_WristRight, "WristRight", JointType_ElbowRight},
		{JointType_HandRight, "HandRight", JointType_WristRight},
		{JointType_HipLeft, "HipLeft", JointType_SpineBase},
		{JointType_KneeLeft, "KneeLeft", JointType_HipLeft},
		{JointType_AnkleLeft, "AnkleLeft", JointType_KneeLeft},
		{JointType_FootLeft, "FootLeft", JointType_AnkleLeft},
		{JointType_HipRight, "HipRight", JointType_SpineBase},
		{JointType_KneeRight, "KneeRight", JointType_HipRight},
		{JointType_AnkleRight, "AnkleRight", JointType_KneeRight},
		{JointType_FootRight, "FootRight", JointType_AnkleRight},
		{JointType_SpineShoulder, "SpineShoulder", JointType_SpineMid},
		{JointType_HandTipLeft, "HandTipLeft", JointType_HandLeft},
		{JointType_ThumbLeft, "ThumbLeft", JointType_WristLeft},
		{JointType_HandTipRight, "HandTipRight", JointType_HandRight},
		{JointType_ThumbRight, "ThumbRight", JointType_WristRight},
	};
}

void write_frame_data(IBodyFrame* frame, std::ofstream& output_file, const std::vector<bone_details>& bone_layout) {
	if (!frame || output_file.bad()) {
		return;
	}

	TIMESPAN relative_time = 0;
	if (FAILED(frame->get_RelativeTime(&relative_time))) {
		return;
	}
	const std::int64_t timestamp = static_cast<std::int64_t>(relative_time);

	IBody* bodies[BODY_COUNT] = {};
	if (FAILED(frame->GetAndRefreshBodyData(BODY_COUNT, bodies))) {
		for (auto& body : bodies) {
			if (body) {
				body->Release();
				body = nullptr;
			}
		}
		return;
	}

	const size_t column_count = bone_layout.size() * 7;
	std::vector<float> column_data(column_count);

	for (int i = 0; i < BODY_COUNT; i++) {
		IBody* body = bodies[i];
		if (!body) {
			continue;
		}

		BOOLEAN is_tracked = FALSE;
		if (FAILED(body->get_IsTracked(&is_tracked)) || !is_tracked) {
			continue;
		}

		UINT64 tracking_id = 0;
		if (FAILED(body->get_TrackingId(&tracking_id))) {
			continue;
		}

		Joint joints[JointType_Count] = {};
		JointOrientation orientations[JointType_Count] = {};
		if (FAILED(body->GetJoints(JointType_Count, joints)) || FAILED(body->GetJointOrientations(JointType_Count, orientations))) {
			continue;
		}

		std::fill(column_data.begin(), column_data.end(), std::numeric_limits<float>::quiet_NaN());

		for (const auto& bone : bone_layout) {
			const JointType joint_type = static_cast<JointType>(bone.ID);
			if (joint_type < 0 || joint_type >= JointType_Count) {
				continue;
			}

			const size_t start_column = bone.first_column;
			if (joints[joint_type].TrackingState == TrackingState_NotTracked) {
				continue;
			}

			column_data[start_column + 0] = joints[joint_type].Position.X;
			column_data[start_column + 1] = joints[joint_type].Position.Y;
			column_data[start_column + 2] = joints[joint_type].Position.Z;
			column_data[start_column + 3] = orientations[joint_type].Orientation.w;
			column_data[start_column + 4] = orientations[joint_type].Orientation.x;
			column_data[start_column + 5] = orientations[joint_type].Orientation.y;
			column_data[start_column + 6] = orientations[joint_type].Orientation.z;
		}

		output_file << timestamp << ',' << tracking_id;
		for (size_t j = 0; j < column_count; j++) {
			output_file << ',' << column_data[j];
		}
		output_file << '\n';
	}

	for (auto& body : bodies) {
		if (body) {
			body->Release();
			body = nullptr;
		}
	}
}

int main(int argc, char* argv[]) {
	if (argc < 3 || argc > 4) {
		cerr << "USAGE: \"" << argv[0] << "\": time_minutes output_file [time_seconds]\n";
		return 1;
	}

	float recording_time = 0.0f;
	try {
		recording_time = std::stof(argv[1]);
	} catch (const std::exception&) {
		cerr << argv[0] << ": Invalid time_minutes\n";
		return 1;
	}

	std::filesystem::path output_file_path(argv[2]);
	{
		if (std::filesystem::exists(output_file_path) && std::filesystem::is_directory(output_file_path)) {
			cerr << argv[0] << ": output_file cannot be a directory\n";
			return 1;
		}

		std::filesystem::path parent_path = output_file_path.parent_path();
		if (!parent_path.empty() && (!std::filesystem::exists(parent_path) || !std::filesystem::is_directory(parent_path))) {
			cerr << argv[0] << ": output directory \"" << parent_path << "\" is invalid\n";
			return 1;
		}
	}

	std::ofstream output_file(output_file_path);
	if (output_file.bad()) {
		cerr << argv[0] << ": output_file could not be opened\n";
		return 1;
	}

	float wait_time = 0.0f;
	if (argc == 4) {
		try {
			wait_time = std::stof(argv[3]);
		} catch (const std::exception&) {
			cerr << argv[0] << ": Invalid wait_time\n";
			return 1;
		}
	}

	printf("Writer starting with a time of %0.2f, to file: ", recording_time);
	cout << output_file_path.string() << std::endl;

	cout << "getting bone layout...\n";
	auto bone_layout = get_bone_layout();

	cout << "writing header...\n";
	output_file << "Kinect Version:,\"2.0\",";

	output_file << "Bone ID,Name,Parent ID\n";
	for (const auto& bone : bone_layout) {
		output_file << bone.ID << ',';
		output_file << bone.name << ',';
		output_file << bone.parent_ID << '\n';
	}

	output_file << ",";
	{
		uint8_t current_column = 0;
		for (auto& bone : bone_layout) {
			bone.first_column = current_column;
			current_column = static_cast<uint8_t>(current_column + 7);
			for (unsigned int i = 0; i < 7; i++) {
				output_file << ',' << bone.name;
			}
		}
	}

	output_file << "\ntimestamp,skeletonID,";
	for (unsigned int i = 0; i < bone_layout.size(); i++) {
		output_file << "x,y,z,qw,qx,qy,qz,";
	}
	output_file << '\n';

	cout << "getting reader...\n";
	reader kinect_reader;
	const HRESULT init_result = kinect_reader.initialize();
	if (FAILED(init_result)) {
		cerr << "Failed to initialize Kinect reader (HRESULT=" << std::hex << init_result << ")\n";
		return 1;
	}

	const auto [event_notifier, body_reader] = kinect_reader.get_notifier();
	if (event_notifier == 0 || body_reader == nullptr) {
		cerr << "Kinect reader did not provide a valid notifier or body reader\n";
		return 1;
	}

	cout << "Finished Kinect Connection...\n";

	cout << "Waiting " << wait_time << "seconds...\n";
	
	if (argc == 4) {
		for(unsigned int time_left = wait_time; time_left != 0; time_left--){
			cout << time_left << '\n';
			std::this_thread::sleep_for(std::chrono::duration<float>(1));
		}
	}

	cout << "Recording!\n";

	const auto stop_time = std::chrono::steady_clock::now() + std::chrono::duration<float, std::ratio<60>>(recording_time);

	while (std::chrono::steady_clock::now() < stop_time) {
		const DWORD wait_result = WaitForSingleObject(reinterpret_cast<HANDLE>(event_notifier), 50);
		if (wait_result != WAIT_OBJECT_0) {
			continue;
		}

		IBodyFrameArrivedEventArgs* event_args = nullptr;
		if (FAILED(body_reader->GetFrameArrivedEventData(event_notifier, &event_args)) || !event_args) {
			continue;
		}

		IBodyFrameReference* frame_ref = nullptr;
		HRESULT frame_result = event_args->get_FrameReference(&frame_ref);
		if (SUCCEEDED(frame_result) && frame_ref) {
			IBodyFrame* body_frame = nullptr;
			frame_result = frame_ref->AcquireFrame(&body_frame);
			if (SUCCEEDED(frame_result) && body_frame) {
				write_frame_data(body_frame, output_file, bone_layout);
				body_frame->Release();
				body_frame = nullptr;
			}
			frame_ref->Release();
			frame_ref = nullptr;
		}

		event_args->Release();
		event_args = nullptr;
	}

	cout << "Done!\n";
	return 0;
}
