# ROS2 Internal Dependency Checker

This clang tool acts on C++ ROS 2 nodes and finds intra-node data dependencies between callbacks.
It is common that e.g. a subscription callback receives a message and modifies the node's state, 
which is later read by e.g. a timer callback.

Since this type of data dependency is not recorded through publish/subscribe calls, we have to uncover
those dependencies separately.

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
sudo apt-get install llvm clang libclang-dev
```

The project can then be build by executing:
```shell
./build.sh
```

## Usage

First, compile your ROS 2 project using colcon with the `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON` argument:
```shell
CC=/usr/bin/clang CXX=/usr/bin/clang++ colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

You **must** build with the same version of clang you use to compile this project.

Finally, execute this tool:
```shell
./dep-check -p /path/to/colcon/build /path/to/node/source.cpp
```