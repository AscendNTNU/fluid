#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "fluid_client");
    ros::spin();

    return 0;
}
