# ROS2 Internal Dependency Analyzer

This clang tool acts on C++ ROS 2 nodes and finds intra-node data dependencies between callbacks.
It is common that e.g. a subscription callback receives a message and modifies the node's state, 
which is later read by e.g. a timer callback.

Since this type of data dependency is not recorded through publish/subscribe calls, 
those dependencies have to be uncovered separately.

This tool parses the source code of ROS 2 nodes and searches for calls to
```c++
rclcpp::create_subscription<message_type>(topic_name, qos, callback);
rclcpp::create_timer(node, clock, period, callback);
rclcpp::create_publisher<message_type>(topic_name, qos);
```
and searches through callback function bodies to find accesses to the node's member fields.
These accesses are categorized into `read`, `write` and `both` if possible.
If the type of access cannot be determined, the user is prompted to categorize the access.

## Setup

Install the following dependencies:
```shell
sudo apt-get update
sudo apt-get install clang-13 libclang1-13 libclang-13-dev libclang-cpp13-dev libclang-cpp13
```
Note that depending on your system AND NVIDIA library versions, 
a different Clang version could be necessary. 
Check [the NVIDIA CUDA Toolkit Docs](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#system-requirements)
for more info.

The project can then be build by executing:
```shell
./build.sh
```

## Usage

First, compile your ROS 2 project using colcon with the `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON` argument:
```shell
CC=/usr/bin/clang CXX=/usr/bin/clang++ colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

Some ROS 2 projects might need small compatibility adjustments of their CMake files and possibly
of their source code (e.g., std:: namespace prefixes, -Werror removals, etc.).
You **must** build with the same version of clang you use to compile this project.

Finally, execute this tool:
```shell
./dep-check -p /path/to/colcon/build /path/to/node/source.cpp
```
Or, for batch execution:
```shell
./analyze_all.bash /path/to/ros/workspace
```

## Further Info

Tested with Ubuntu 22.04, Clang 13, 
[Autoware.Universe (commit 20fd431)](https://github.com/autowarefoundation/autoware/commit/20fd431aea2e80f307341418b474d6023507988b),
[ROS 2 Humble](https://docs.ros.org/en/humble/index.html),
CUDA 11.6 with CuDNN 8.4.0.27-1+cuda11.6 and TensorRT 8.2.4-1+cuda11.4 on x86.