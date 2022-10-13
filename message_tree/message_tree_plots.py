from typing import List

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes
from matplotlib.figure import Figure

from message_tree.message_tree_structure import E2EBreakdownItem


def e2e_breakdown_type_hist(items: List[E2EBreakdownItem]):
    """
    Given a list of e2e breakdown instances of the form `("<type>", <duration>)`, plots a histogram for each encountered
    type.
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


def e2e_breakdown_inst_stack(*paths: List[E2EBreakdownItem]):
    fig: Figure
    ax: Axes
    fig, ax = plt.subplots(num="E2E instance breakdown stackplot")
    fig.suptitle("Detailed E2E Latency Path Breakdown")

    bottom = 0
    for i in range(len(paths[0])):
        e2e_items = [path[i] for path in paths]
        durations = np.array([item.duration for item in e2e_items])
        ax.bar(range(len(paths)), durations, bottom=bottom)
        bottom = durations + bottom

    return fig
