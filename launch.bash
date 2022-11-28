#!/bin/bash

# This script launches all components of the scenario runner on a set of pre-defined machines, which can be configured
# using the environment variables in the top part of this file.

#################################################
# Config Section
#################################################

aw_hostname=edgar-sim-dev
aw_username=adlink
aw_rootdir=/home/$aw_username/Max_MA

sim_hostname=edgar-gpu
sim_username=sim
sim_rootdir=/home/$sim_username/Max_MA

#################################################
# Script Execution
#################################################

ssh -t ${sim_username}@${sim_hostname} "screen -S sim_orchestrator -d -m ${sim_workdir}/scenario_runner/worker.bash sim"
ssh -t ${aw_username}@${aw_hostname} "screen -S aw_orchestrator -d -m ${aw_workdir}/scenario_runner/worker.bash aw"