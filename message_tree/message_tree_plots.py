from typing import List

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes
from matplotlib.figure import Figure

from message_tree.message_tree_structure import E2EBreakdownItem


def e2e_breakdown_type_hist(items: List[E2EBreakdownItem]):
    """
    Given a list of e2e breakdown instances of the form `("<type>", <duration>)`, plots a histogram for each encountered type.
    Be careful not to mix items of different points in the DFG (i.e. do NOT input dataflows here).
    :param items: The list of items to be turned into a histogram
    :return: The figure of the plot
    """
    plot_types = ("dds", "idle", "cpu")
    assert all(item.type in plot_types for item in items)

    fig: Figure
    fig, axes = plt.subplots(1, 3, num="E2E type breakdown histograms")
    fig.suptitle("E2E Latency Breakdown by Resource Type")

    for type, ax in zip(plot_types, axes):
        ax: Axes

        durations = [item.duration for item in items if item.type == type]

        ax.set_title(type)
        ax.hist(durations)
        ax.set_xlabel("Duration [s]")
        ax.set_ylabel("Occurrences")

    return fig


def e2e_breakdown_stack(*paths: List[E2EBreakdownItem]):
    """
    Plot a timeseries of stacked DDS/Idle/CPU latencies from `paths`.
    Each path has to be the same length
    :param paths: The E2E paths to plot
    :return: The figure of the plot
    """
    fig: Figure
    ax: Axes
    fig, ax = plt.subplots(num="E2E type breakdown stackplot")
    fig.suptitle("Detailed E2E Latency Path Breakdown")

    if not paths:
        return fig

    plot_types = ("dds", "idle", "cpu")

    type_indices = {type: [i for i, item in enumerate(paths[0]) if item.type == type] for type in plot_types}
    type_durations = {}

    for type in plot_types:
        durations = [sum([item.duration for i, item in enumerate(path) if i in type_indices[type]]) for path in paths]
        durations = np.array(durations)
        type_durations[type] = durations

    labels, duration_arrays = zip(*sorted(list(type_durations.items()), key=lambda pair: pair[1].var(), reverse=False))
    ax.stackplot(range(len(paths)), *duration_arrays, labels=labels)
    ax.legend()
    return fig
