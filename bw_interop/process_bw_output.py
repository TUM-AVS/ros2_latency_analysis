import pandas as pd
import tables as tb
import numpy as np


def bytes_str(bytes):
    if bytes >= 1024**2:
        return f"{bytes/(1024**2):.2f} MiB"
    if bytes >= 1024:
        return f"{bytes/1024:.2f} KiB"
    return f"{bytes:.0f} B"


def get_topic_messages(h5_filename: str):
    topic_messages = {}

    with tb.open_file(h5_filename, root_uep="/messages") as f:
        for node in f.list_nodes("/"):
            topic = node.title
            messages = pd.DataFrame.from_records(node[:])
            topic_messages[topic] = messages
            # if len(messages) >= 2:
            #     total_data = np.sum(messages["size"])
            #     print(f'{len(messages):>5d}m, {bytes_str(total_data):>10s}, '
            #     f'{bytes_str(total_data / (np.max(messages["timestamp"]) - np.min(messages["timestamp"]))):>10s}/s, '
            #     f'{topic}')

    return topic_messages


def get_topic_stats(topics_dict: dict):
    records = []
    for topic, messages in topics_dict:
        total_data = np.sum(messages["size"])
        records.append({
                "topic": topic,
                "count": len(messages),
                "total_data": total_data,
                "bandwidth": total_data / (np.max(messages["timestamp"]) - np.min(messages["timestamp"])),
                "min_size": np.min(messages["size"]),
                "avg_size": np.mean(messages["size"]),
                "max_size": np.max(messages["size"])
        })

    return pd.DataFrame.from_records(records)
