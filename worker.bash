#!/bin/bash

cd $(dirname $(basename -- $0))
role=$1
export ROS_DOMAIN_ID=69
source install/setup.bash

case $role in
    aw)
    echo "Launching Autoware orchestrator"
    ros2 run awsim_scenario_runner aw_orchestrator &
    python3 scenario_runner.py -c config/aw_awsim.yml ROS_DOMAIN_ID:=69
    ;;
    sim)
    echo "Launching Simulator orchestrator"
    ros2 run awsim_scenario_runner sim_orchestrator
    ;;
    *)
    echo "Invalid argument role=$role"
    ;;
esac
