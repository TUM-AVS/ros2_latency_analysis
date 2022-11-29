#!/bin/bash

# This script launches all components of the scenario runner on a set of pre-defined machines, which can be configured
# using the environment variables in the top part of this file.

#################################################
# Config Section
#################################################

aw_hostname=edgar-sim-dev
aw_username=adlink
aw_rootdir=/home/$aw_username/Max_MA

sim_hostname=sim-SYS-7049GP-TRT
sim_username=sim
sim_rootdir=/home/$sim_username/Max_MA

#################################################
# Script Execution
#################################################

ssh -t ${sim_username}@${sim_hostname} "screen -L -Logfile /home/sim/Max_MA/scenario_runner/worker.log -S sim_orchestrator -d -m /home/sim/Max_MA/scenario_runner/worker.bash sim"
ssh -t ${aw_username}@${aw_hostname} "screen -L -Logfile /home/adlink/Max_MA/scenario_runner/worker.log -S aw_orchestrator -d -m /home/adlink/Max_MA/scenario_runner/worker.bash aw"
