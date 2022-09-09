# MA AWSIM Scenario Runner

Run scenarios reproducibly in AWSIM.

## Installation

```shell
git clone https://gitlab.lrz.de/schmeller/ma-awsim-scenario-runner.git
cd ma-awsim-scenario-runner
pip install -r requirements.txt
```

## Usage

In the `scenarios` folder, choose or create a scenario you want to run.
Each scenario is a folder with a `config.yml` file and, optionally, additional assets.
The config file contains:
* start and goal pose
* end condition (e.g. time since start)
* AWSIM host
* (optionally) rosbag files for pedestrians etc.

Start the runner with
```shell
./scenario_runner.py <scenario-name>
```

The runner will automatically launch
* Autoware 
* AWSIM
* ROS 2 tracing
* ROS 2 multitopic bw
* perf

in the correct order and will produce a new folder named `<scenario-name>_<timestamp>`.
This folder will contain:
* System information (hostname, kernel, CPU, RAM, PCIe info)
* Environment variable values
* Recorded tracing data
* Recorded message sizes
* Recorded perf data
