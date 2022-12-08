# Self Contained Experimental Setup

This repository serves as a container for the whole experimental setup needed to reproduce the results of the Thesis.
Specifically, the setup includes:
* [Autoware Core/Universe](https://github.com/autowarefoundation/autoware)
* [ROS2 Tracing](https://github.com/ros2/ros2_tracing)
* [ROS2 Multitopic](https://gitlab.lrz.de/schmeller/ros2multitopic)
* [Autoware Scenario Runner](https://gitlab.lrz.de/schmeller/ma-autoware-scenario-runner)
* [AWSIM (modified to exclude traffic, stoplights and pedestrians)](https://github.com/mojomex/AWSIM)
* [Nishi-Shinjuku map files](https://github.com/tier4/AWSIM/releases/download/v1.0.0/nishishinjuku_autoware_map.zip)

## Setup

**It is necessary to clone this repo into your home directory and to change its name to `Max_MA`!**
```bash
cd ~
git clone https://gitlab.lrz.de/schmeller/ma-experiment-workspace.git Max_MA
cd Max_MA
./setup.bash
```
