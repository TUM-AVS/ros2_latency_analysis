# MA Autoware Trace Analysis

Automatically extract data dependencies and end-to-end (E2E) latencies from ROS2 trace data.

## Prerequisites
* Python 3.10 or newer (this is crucial!)
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

Make sure that ROS2 Tracing and Tracetools Analysis were compiled with `colcon build [...] --symlink-install [...]`. Without `--symlink-install`, the build folder structure will be incorrect and the libraries required by this tool cannot be located.

## Usage

The `trace_analysis.ipynb` notebook is the entry point for users.
Configure the notebook according to the comments in the user settings cell.
Settings can either be changed in the notebook or via environment variables:
```python
# In the notebook (User Settings cell near the top):
TR_PATH = "path/to/trace/dir"
E2E_ENABLED = True
...
```

```shell
# In the shell of your choice (choose Bash!):
# Each setting is named the same as in the notebook but prefixed by "ANA_NB_".
ANA_NB_TR_PATH="path/to/trace/dir"
ANA_NB_E2E_ENABLED="True"
```

You can run the notebook via the "Run All" command in Jupyter or you can execute it headless
from the command line:
```shell
jupyter nbconvert --to notebook --execute trace-analysis.ipynb
```

nbconvert can also be called from Python directly, 
read more info on nbconvert [here](https://nbconvert.readthedocs.io/en/latest/execute_api.html).

The output files are found in the configured output dir (default: `out/`). Inputs are processed and cached in `cache/`.
