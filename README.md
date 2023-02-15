# ROS 2 Data Flow Analysis

This repository contains tools to
1. execute ROS 2 programs repeatably with tracing and resource usage trackig
2. extract ROS 2 node-internal data dependencies from C++ source code
3. analyze end-to-end latencies and their breakdowns, given data from 1. and 2.

The tools are found in the branches of this repository.

## Setup

### Runner Framework

For data acquisition / simulation runs, up to three hosts can be utilized.
For all those hosts, the setup is as follows:

```bash
    git clone -b runner-framework https://github.com/TUM-AVS/ros2_latency_analysis.git runner_framework
    cd runner_framework
    ./setup.bash
```

### Code Analysis

C++ dependency analysis has to be done on the same code base that is run in the data acquisition step
but the machine can differ.
The setup is as follows:

```bash
    git clone -b code-analysis https://github.com/TUM-AVS/ros2_latency_analysis.git code_analysis
    cd code_analysis
```

Follow the instructions in the `README.md` file in that directory.

### Dataflow Analysis

Dataflow analysis has to be done with the same ROS 2 version as the above steps but the machine can differ.
The setup is as follows:

```bash
    git clone -b dataflow-analysis https://github.com/TUM-AVS/ros2_latency_analysis.git dataflow_analysis
    cd dataflow_analysis
```

Follow the instructions in the `README.md` file in that directory.
