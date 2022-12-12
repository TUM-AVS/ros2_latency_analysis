#!/bin/bash

# This script launches all components of the scenario runner on a set of pre-defined machines, which can be configured
# using the environment variables in the top part of this file.

#################################################
# Config Section
#################################################

# The string after the :- is the default value if the environment variable before it is not set
aw_hostname=$SC_AW_HOSTNAME
aw_username=$SC_AW_USERNAME

sim_hostname=${SC_SIM_HOSTNAME:-sim-SYS-7049GP-TRT}
sim_username=${SC_SIM_USERNAME:-sim}

# Will be generated if not present
ssh_id=${SC_SSH_ID:-~/.ssh/id_max_ma}

cfg_path=$SC_CFG_PATH

#################################################
# Properties based on config
#################################################

# Has to be an absolute path with no environment variables etc. other than $sim_username
sim_rootdir=/home/$sim_username/Max_MA
# Has to be an absolute path with no environment variables etc. other than $aw_username
aw_rootdir=/home/$aw_username/Max_MA
# Local path to the SSH ID file (without .pub) to be used for remote access to the workers


#################################################
# Interactive First-Time Setup 
# (runs non-interactively if already done)
#################################################

if [ ! -f "$ssh_id" ]
then
    echo "[LAUNCHER] Generating SSH keypair for automated access to worker machines."
    echo "[LAUNCHER] LEAVE THE PASSWORD BLANK (just press enter)"
    ssh-keygen -f "$ssh_id"
fi

ssh-copy-id -i "$ssh_id" "${sim_username}"@"${sim_hostname}"
ssh-copy-id -i "$ssh_id" "${aw_username}"@"${aw_hostname}"

#################################################
# Script Execution
#################################################

pids=()
(
    ssh -i "$ssh_id" "${sim_username}"@"${sim_hostname}" "rm -f ${sim_rootdir}/scenario_runner/worker.log" &&
    ssh -tt -i "$ssh_id" "${sim_username}"@"${sim_hostname}" "screen -L -Logfile ${sim_rootdir}/scenario_runner/worker.log -S sim_orchestrator timeout -k20 260 ${sim_rootdir}/scenario_runner/worker.bash sim $cfg_path" > /dev/null &&
    ssh -i "$ssh_id" "${sim_username}"@"${sim_hostname}" "pkill --signal SIGKILL -f 'AWSIM|sim_orchestrator'"
) &
pids+=($!)
echo "[LAUNCHER] Launched sim worker on ${sim_username}@${sim_hostname}"
(
    ssh -i "$ssh_id" "${aw_username}"@"${aw_hostname}" "rm -f ${aw_rootdir}/scenario_runner/worker.log" &&
    ssh -tt -i "$ssh_id" "${aw_username}"@"${aw_hostname}" "screen -L -Logfile ${aw_rootdir}/scenario_runner/worker.log -S aw_orchestrator timeout -k20 260 ${aw_rootdir}/scenario_runner/worker.bash aw $cfg_path" > /dev/null &&
    ssh -i "$ssh_id" "${aw_username}"@"${aw_hostname}" "pkill --signal SIGKILL -f 'ros|http.server|aw_orchestrator'"
) &
pids+=($!)
echo "[LAUNCHER] Launched aw worker on ${aw_username}@${aw_hostname}"

if [ "$1" = "rviz" ]
then
    echo "[LAUNCHER] Launching RVIZ"
    source ../autoware/install/setup.bash &&
    export ROS_DOMAIN_ID=69 &&
    rviz2 -d ../autoware/install/autoware_launch/share/autoware_launch/rviz/autoware.rviz -s ../autoware/install/autoware_launch/share/autoware_launch/rviz/image/autoware.png > /dev/null &
fi

echo "[LAUNCHER] Waiting for workers to finish..."
wait "${pids[@]}"

echo "[LAUNCHER] Done."

artifacts_filename=artifacts_$(date "+%Y%m%d_%H%M%S").zip
echo "[LAUNCHER] Copying artifacts from aw worker to '$artifacts_filename'..."
scp -i "$ssh_id" "${aw_username}"@"${aw_hostname}":"${aw_rootdir}"/scenario_runner/artifacts.zip "$artifacts_filename" || exit 1
echo "[LAUNCHER] Done."
exit 0
