FROM ros:kinetic-ros-base
MAINTAINER Ascend NTNU "www.ascendntnu.no"

ENV ROS_WORKSPACE_PATH=/opt/catkin_ws
ENV ROS_PACKAGE_NAME=FLUID_FSM

RUN apt-get update -qq && apt-get install -yqq \
    build-essential \
    ros-kinetic-mavros \
    ros-kinetic-mavros-extras \
    ros-kinetic-tf2


RUN mkdir -p $ROS_WORKSPACE_PATH/src
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; catkin_init_workspace $ROS_WORKSPACE_PATH/src'

# Run caktin_make once without building any packages to create the setup.bash
# RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; cd $ROS_WORKSPACE_PATH; catkin_make'

COPY ./ $ROS_WORKSPACE_PATH/src/$ROS_PACKAGE_NAME/
WORKDIR $ROS_WORKSPACE_PATH/src
RUN git clone --depth 1 -b master https://github.com/AscendNTNU/ascend_msgs.git
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; cd $ROS_WORKSPACE_PATH; catkin_make'
RUN /bin/bash -c 'echo source $ROS_WORKSPACE_PATH/devel/setup.bash >> /root/.bashrc'
