//
// Created by simengangstad on 21.02.19.
//

#include <ros/ros.h>
#include <std_msgs/String.h>

const char* lastCommand;

void commandCallback(const std_msgs::String::ConstPtr& string) {
    const char* newCommand = string->data.c_str();

    if (lastCommand != newCommand) {

        std::cout << "New command: " << newCommand << std::endl;

        if (newCommand == "alpha") {

        }
        else if (newCommand == "bravo") {

        }

        lastCommand = newCommand;
    }
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "ai_voice_client");
    ros::NodeHandle node_handle;

	node_handle.subscribe<std_msgs::String>("ai/voice/command", 10, commandCallback);

	ros::spin();

    return 0;
}