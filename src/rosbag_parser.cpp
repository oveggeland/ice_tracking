/*
This node plays back messages from a bag files in a folder.
*/

#include <ros/ros.h>
#include <filesystem>


void playback(const std::filesystem::path& bagfile) {
    // Build command to launch playback node
    std::string command = "rosbag play " + bagfile.string() + " -i";
    
    // Execute command
    int ret = std::system(command.c_str());
    if (ret != 0) {
        ros::shutdown(); // The command fails on CTRL+C
    }
}

void parseBagFiles(const std::filesystem::path bagpath){
    std::vector<std::filesystem::path> files;
    std::copy(std::filesystem::directory_iterator(bagpath), std::filesystem::directory_iterator(), std::back_inserter(files));
    std::sort(files.begin(), files.end());

    for (const std::string& filename : files) {
        playback(filename);

        if (!ros::ok()){
            break;
        }
    }
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosbag_parsing_node");
    ros::NodeHandle nh("~");

    std::string bagpath; // Path to folder of bag files
    if (!nh.getParam("/bagpath", bagpath)){
        ROS_ERROR("Failed to retrieve bag path.");
        return 1;
    }

    parseBagFiles(bagpath);

    return 0;
}