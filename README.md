# Self Contained Experimental Setup

This repository serves as a container for the whole experimental setup needed to reproduce the results of the paper.

Prerequisites:
* Ubuntu 20.04
* ROS 2 Galactic

Specifically, the setup includes:
* [Autoware Core/Universe](https://github.com/autowarefoundation/autoware)
* [ROS2 Tracing](https://github.com/ros2/ros2_tracing)
* [Autoware Scenario Runner](https://github.com/TUM-AVS/ros2_latency_analysis/tree/scenario-runner)   
* [AWSIM (modified to exclude traffic, stoplights and pedestrians)](https://github.com/TUM-AVS/ros2_latency_analysis/tree/awsim)
* [Nishi-Shinjuku map files](https://github.com/tier4/AWSIM/releases/download/v1.0.0/nishishinjuku_autoware_map.zip)

## Setup

```bash
git clone -b runner-framework https://github.com/TUM-AVS/ros2_latency_analysis.git runner_framework
cd runner_framework
./setup.bash [--sim] [--aw]
```

If neither `--sim` nor `--aw` is given (to install AWSIM and/or Autoware respectively), everything is installed.
The scenario runner is always installed. ros2_tracing is only installed with Autoware, the map only with AWSIM.