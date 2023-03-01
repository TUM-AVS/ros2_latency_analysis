# MA Autoware Scenario Runner

Run Autoware scenarios reproducibly.

## Installation

Do not set up manually.
Use the `setup.bash` script in the `runner-framework` branch instead.

## Usage

### Non-distributed
```shell
./scenario_runner.py [-c path/to/config.yml] ROS_DOMAIN_ID:=<num> [ENV_VAR_1:=val1 [ENV_VAR_2:=val2]]
```

`ROS_DOMAIN_ID` has to be present to prevent the runner interfering with other people's ROS systems.
The environment variables given are passed on to every task started from the config.

The `config` directory contains a set of launch configurations to choose from.
Each configuration is a YAML file containing a set of tasks and other settings.

Current configs:
* `aw_replay.yml` launches the Autoware replay sim and runs it for 30s with tracing and message size recording
* `collect_system_info.yml` collects hardware and software confiugration info and Autoware repo state

Starting the runner with
```shell
./scenario_runner.py -c config/aw_replay.yml ROS_DOMAIN_ID:=33
```

The runner will automatically launch
* Autoware
* AWSIM
* ROS 2 tracing
* ROS 2 multitopic bw

in the correct order and will produce a new folder named `artifacts/aw_replay`.
This folder will contain:
* Recorded tracing data
* Recorded message sizes
* Recorded perf data
* Autoware logs

## Distributed

To run the program across multiple PCs, with one Management (M) machine, AWSIM (S) and Autoware (A) machine, do this on the (M) machine:

```shell
./launch.bash
```
With the environment variables mentioned in that file changed to the correct hosts/paths.
This might require you to enter the hosts' passwords the first time.
This repository has to be installed and up-to-date as described above on all hosts.
The artifacts from (A) will be zipped and copied to (M).

Use 
```shell
./batch_measure.py
```
on (M) to measure each scenario multiple times, and to measure multiple scenarios after another.
Follow the instructions in that file for this.
## Configuration Format

```yaml
# Environment variables set for all tasks
environment:
  VAR_1: "value 1"
# A list of tasks to be started
tasks:
  # Special task names like autoware or rosbag behave differently (they have to print console outputs before they are considered running)
  task_name:
    # Bash commands, will be executed in order
    commands:
      - "cd .."
      - "source ./x.sh"
      - "run xyz"
    # (Optional) This path (can be a directory) will be copied to a common output directory of the runner
    artifact_location: "path/to/task/artifacts"
    # (Optional) This task will only be started once its dependencies are running
    start_deps:
      - task_name_1
      - task_name_2
# (Optional) Time in seconds after which every job is terminated.
# The time starts counting after all jobs are running
runtime_s: 30
```
