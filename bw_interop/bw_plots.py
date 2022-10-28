import matplotlib.pyplot as plt
import numpy as np


def dds_lat_msg_size_scatter(topic_name, topic_dds_latencies, topic_msg_sizes):
    plt.close("dds_lat_msg_size_scatter")
    fig, ax = plt.subplots(num="dds_lat_msg_size_scatter", dpi=300, figsize=(15, 7))
    ax: plt.Axes
    fig: plt.Figure
    fig.suptitle(f"Correlation of Message Size and DDS Latency\nfor {topic_name}")

    ax.scatter(np.array(topic_dds_latencies) * 1e6, topic_msg_sizes)
    ax.set_xlabel("Message Size [B]")
    ax.set_ylabel("DDS Latency [µs]")

    return fig
