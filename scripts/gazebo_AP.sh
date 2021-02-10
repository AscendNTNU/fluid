#!/usr/bin/env bash

cd $1
gazebo --verbose worlds/iris_arducopter_runway.world \
    |./scripts/start_ardupilot.sh
