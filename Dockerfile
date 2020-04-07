FROM ros:melodic-ros-base
MAINTAINER Ascend NTNU "www.ascendntnu.no"

ENV ROS_WORKSPACE_PATH=/opt/catkin_ws
ENV ROS_PACKAGE_NAME=fluid

RUN apt-get update -qq && apt-get install -yqq \
    build-essential \
    ros-melodic-mavros \
    ros-melodic-mavros-extras \
    cmake \
    python-catkin-pkg \
    python-catkin-tools \
    python-empy \
    python-nose \
    libgtest-dev \
    ros-melodic-tf2-geometry-msgs \
    ros-melodic-tf2


RUN mkdir -p $ROS_WORKSPACE_PATH/src
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_init_workspace $ROS_WORKSPACE_PATH/src'

# Run caktin_make once without building any packages to create the setup.bash
# RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; cd $ROS_WORKSPACE_PATH; catkin_make'

COPY ./ $ROS_WORKSPACE_PATH/src/$ROS_PACKAGE_NAME/
WORKDIR $ROS_WORKSPACE_PATH/src
RUN git clone --depth 1 -b master https://github.com/AscendNTNU/ascend_msgs.git
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd $ROS_WORKSPACE_PATH; catkin build'
RUN /bin/bash -c 'echo source $ROS_WORKSPACE_PATH/devel/setup.bash >> /root/.bashrc'
