#!/bin/bash

rootdir="$(dirname "${BASH_SOURCE[0]}")"
rootdir="$(realpath "${DIR}")"
cd "$rootdir"/scenario_runner || exit 1
# Either 'aw' or 'sim'
role=$1
# The path to a config (.yml) file for the scenario runner
conf=$2
# The ROS donain ID to run all ROS commands in
domain_id=$3
export ROS_DOMAIN_ID=$domain_id
source install/setup.bash

# Ger scenario name by removing path and file extension, leaving only the stem (e.g. "./config/aw_awsim.yml" --> "aw_awsim")
scenario_name=$(basename "$conf" .yml)

case $role in
    aw)
    echo "Launching Autoware orchestrator on scenario ${scenario_name}"
    rm -rf "artifacts"
    ros2 run awsim_scenario_runner aw_orchestrator &
    python3 scenario_runner.py -c "$conf" ROS_DOMAIN_ID:="$domain_id"
    ;;
    sim)
    echo "Launching Simulator orchestrator"
    ros2 run awsim_scenario_runner sim_orchestrator
    ;;
    *)
    echo "Invalid argument role=$role"
    ;;
esac
