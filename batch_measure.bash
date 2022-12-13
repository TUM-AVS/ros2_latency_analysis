#!/bin/bash

set -e

date -u -Iseconds >> launch.log
echo "### $0 ###" | tee -a launch.log

# Format: SC_AW_HOSTNAME;SC_AW_USERNAME;SC_CFG_PATH
scenarios=(
        "edgar-hil-x86;edgar;./config/aw_awsim.yml"
        "edgar-sim-dev;adlink;./config/aw_awsim.yml"
        "edgar-hil-x86;edgar;./config/aw_awsim_taskset_00-15.yml"
        "edgar-sim-dev;adlink;./config/aw_awsim_taskset_00-15.yml"
        "edgar-hil-x86;edgar;./config/aw_awsim_taskset_00-07.yml"
        "edgar-sim-dev;adlink;./config/aw_awsim_taskset_00-07.yml"
        "edgar-hil-x86;edgar;./config/aw_awsim_taskset_00-03.yml"
        "edgar-sim-dev;adlink;./config/aw_awsim_taskset_00-03.yml"
)

reps=3

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
	        ./launch.bash
        done
done

echo "### done ###" | tee -a launch.log

