# MA Autoware Trace Analysis

Automatically extract data dependencies and end-to-end (E2E) latencies from ROS2 trace data.

## Prerequisites
* Python 3.10
* [JupyterLab](https://jupyter.org/install#jupyterlab) or [Jupyter Notebook](https://jupyter.org/install#jupyter-notebook)
* [ROS2 Tracing](https://github.com/ros2/ros2_tracing)
* [Tracetools Analysis](https://gitlab.com/ros-tracing/tracetools_analysis)
* `python3-babeltrace` and `python3-lttng`, e.g. via `sudo apt install`, **for Python 3.10** (this requires either Ubuntu 22.04 or custom installation)

## Installation

```shell
# Make sure you are running Python 3.10 and the corresponding pip:
python3.10 -V
pip3.10 -V

pip3.10 install -r requirements.txt
```

## Usage
