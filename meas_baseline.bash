#!/bin/bash

set -e

date -u -Iseconds >> launch.log
echo "### baseline.bash ###" | tee -a launch.log

export SC_AW_HOSTNAME=edgar-hil-x86
export SC_AW_USERNAME=edgar
export SC_CFG_PATH=./config/aw_awsim.yml

for i in {1..10}
do
	echo "============== X86 $i / 10 " | tee -a launch.log
	./launch.bash
done

export SC_AW_HOSTNAME=edgar-sim-dev
export SC_AW_USERNAME=adlink
export SC_CFG_PATH=./config/aw_awsim.yml

for i in {1..10}
do
        echo "============== ARM $i / 10" | tee -a launch.log
        ./launch.bash
done

echo "### done ###" | tee -a launch.log

