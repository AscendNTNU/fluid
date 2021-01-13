#!/usr/bin/env bash

cd $1
./binary/ascend_simulator.x86_64 \
    |./scripts/start_ardupilot.sh
 