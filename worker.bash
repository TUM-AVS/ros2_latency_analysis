#!/bin/bash

cd "$HOME"/Max_MA/scenario_runner || exit 1
# Either 'aw' or 'sim'
role=$1
# The path to a config (.yml) file for the scenario runner
conf=$2
export ROS_DOMAIN_ID=69
source install/setup.bash

# Ger scenario name by removing path and file extension, leaving only the stem (e.g. "./config/aw_awsim.yml" --> "aw_awsim")
scenario_name=$(basename "$conf" .yml)

case $role in
    aw)
    echo "Launching Autoware orchestrator on scenario ${scenario_name}"
    rm -rf "artifacts/${scenario_name}"
    ros2 run awsim_scenario_runner aw_orchestrator &
    python3 scenario_runner.py -c "$conf" ROS_DOMAIN_ID:=69
    rm artifacts.zip
    zip -r1 artifacts.zip artifacts/"${scenario_name}"
    ;;
    sim)
    echo "Launching Simulator orchestrator"
    ros2 run awsim_scenario_runner sim_orchestrator
    ;;
    *)
    echo "Invalid argument role=$role"
    ;;
esac
