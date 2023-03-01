# MA Autoware Trace Analysis

Automatically extract data dependencies and end-to-end (E2E) latencies from ROS2 trace data and source code.

## Prerequisites
* Python 3.10 or newer (this is crucial!)
* [JupyterLab](https://jupyter.org/install#jupyterlab) or [Jupyter Notebook](https://jupyter.org/install#jupyter-notebook)
* [ROS2 Tracing](https://github.com/ros2/ros2_tracing)
* [Tracetools Analysis](https://gitlab.com/ros-tracing/tracetools_analysis)
* `python3-babeltrace` and `python3-lttng`, e.g. via `sudo apt install`, **for Python 3.10** (this requires either Ubuntu 22.04 or custom installation)

## Requirements for Optional Features
* `ros2_internal_dependency_analyzer` for finding intra-node dependencies via static analysis, which can then be
  utilized by this tool
* `ros2 multitopic bw` for recording message sizes which can then be processed by this tool


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
# For strings, quotes have to be included IN THE VARIABLE VALUE, i.e. double quotation "'...'" 
# has to be used in most cases
ANA_NB_TR_PATH="'path/to/trace/dir'"
ANA_NB_E2E_ENABLED="True"
```

You can run the notebook via the "Run All" command in Jupyter or you can execute it headless
from the command line:
```shell
jupyter nbconvert --to notebook --execute trace-analysis.ipynb
```

The notebook also supports invocation via [Papermill](https://papermill.readthedocs.io/en/latest/).

The output files are found in the configured output dir (default: `out/`). 
Inputs are processed and cached in `cache/`.

## Outputs

The `plot_e2es_violin_XYZ.csv` files correspond to the latencies for each recorded dataflow on path item XYZ.
The labels for these path items are found in `plot_e2es_violin_labels.csv`.
The latency types (DDS, Idle, Computation) for these path items are found in `plot_e2es_violin_types.csv`.
`calc_times.csv` provides calculation times for all callbacks on the path (not only until publication, which is the case for the computation times above).


## Known Issues

* The histograms output in the notebook for Idle, Computation and DDS times, do not aggregate these values over the end-to-end path. Thus, the latencies of all path elements are added into the same histogram.