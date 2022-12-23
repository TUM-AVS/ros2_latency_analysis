#!/bin/bash

set -e

date -u -Iseconds >> launch.log
echo "### $0 ###" | tee -a launch.log

# Format: SC_AW_HOSTNAME;SC_AW_USERNAME;SC_CFG_PATH
scenarios=(
        "edgar-hil-x86;edgar;./config/aw_awsim_sched_rr.yml"
        "edgar-hil-x86;edgar;./config/aw_awsim_sched_fifo.yml"
        "edgar-hil-x86;edgar;./config/aw_awsim.yml"
        "edgar-hil-x86;edgar;./config/aw_awsim_taskset_00-15.yml"
        "edgar-hil-x86;edgar;./config/aw_awsim_taskset_00-07.yml"
        "edgar-hil-x86;edgar;./config/aw_awsim_taskset_00-03.yml"
        "edgar-hil-x86;edgar;./config/aw_awsim_messages.yml"
)

reps=3

if [ "$1" = "reboot" ]
then
	#ssh -i ~/.ssh/id_max_ma edgar@edgar-hil-x86 "sudo reboot" || true
	ssh -i ~/.ssh/id_max_ma adlink@edgar-sim-dev "sudo reboot" || true
	sleep 10m
fi

for scenario in "${scenarios[@]}"
do
        SC_AW_HOSTNAME=$(cut -d';' -f1 <<<"$scenario")
        SC_AW_USERNAME=$(cut -d';' -f2 <<<"$scenario")
        SC_CFG_PATH=$(cut -d';' -f3 <<<"$scenario")

        export SC_AW_HOSTNAME
        export SC_AW_USERNAME
        export SC_CFG_PATH

	for ((i=1;i<=reps;i++))
        do
	        echo "============== $i / $reps | $SC_AW_USERNAME@$SC_AW_HOSTNAME ($SC_CFG_PATH)" | tee -a launch.log
	        ./launch.bash | tee -a launch.log
        done
done

echo "### done ###" | tee -a launch.log

