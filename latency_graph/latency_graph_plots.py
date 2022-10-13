import math
import re

import graphviz as gv

import latency_graph.latency_graph_structure as lg
from matching.subscriptions import sanitize
from tracing_interop.tr_types import TrContext

NODE_COLORS = {
        "sensing": {"fill": "#e1d5e7", "stroke": "#9673a6"},
        "localization": {"fill": "#dae8fc", "stroke": "#6c8ebf"},
        "perception": {"fill": "#d5e8d4", "stroke": "#82b366"},
        "planning": {"fill": "#fff2cc", "stroke": "#d6b656"},
        "control": {"fill": "#ffe6cc", "stroke": "#d79b00"},
        "system": {"fill": "#f8cecc", "stroke": "#b85450"},
        "vehicle_interface": {"fill": "#b0e3e6", "stroke": "#0e8088"},
        None: {"fill": "#f5f5f5", "stroke": "#666666"}
}

NODE_NAMESPACE_MAPPING = {
        'perception': 'perception',
        'sensing': 'sensing',
        'planning': 'planning',
        'control': 'control',
        'awapi': 'system',
        'autoware_api': 'system',
        'map': 'system',
        'system': 'system',
        'localization': 'localization',
        'robot_state_publisher': None,
        'aggregator_node': None,
        'pointcloud_container': 'sensing',
}


def plot_latency_graph_full(lat_graph: lg.LatencyGraph, tr: TrContext, filename: str):
    # Compare with: https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/node-diagram/

    g = gv.Digraph('G', filename="latency_graph.gv",
                   node_attr={'shape': 'plain'},
                   graph_attr={'pack': '1'})
    g.graph_attr['rankdir'] = 'LR'

    def plot_hierarchy(gv_parent, lg_node: lg.LGHierarchyLevel, **subgraph_kwargs):
        if lg_node.name == "[NONE]":
            return

        print(f"{'  ' * lg_node.full_name.count('/')}Processing {lg_node.name}: {len(lg_node.callbacks)}")
        with gv_parent.subgraph(name=f"cluster_{lg_node.full_name.replace('/', '__')}", **subgraph_kwargs) as c:
            c.attr(label=lg_node.name)
            for cb in lg_node.callbacks:
                if isinstance(cb, lg.LGTrCallback):
                    tr_cb = cb.cb
                    try:
                        sym = tr.callback_symbols.by_id.get(tr_cb.callback_object)
                        pretty_sym = repr(sanitize(sym.symbol))
                    except KeyError:
                        pretty_sym = cb.name
                    except TypeError:
                        pretty_sym = cb.name
                else:
                    pretty_sym = cb.name

                pretty_sym = pretty_sym.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")

                c.node(cb.id(),
                       f'<<table BORDER="0" CELLBORDER="1" CELLSPACING="0"><tr><td port="in"></td><td>{pretty_sym}</td><td port="out"></td></tr></table>>')

            for ch in lg_node.children:
                plot_hierarchy(c, ch, **subgraph_kwargs)

    def plot_lg(graph: lg.LatencyGraph):
        for top_level_node in graph.top_node.children:
            colors = NODE_COLORS[NODE_NAMESPACE_MAPPING.get(top_level_node.name)]
            plot_hierarchy(g, top_level_node, graph_attr={'bgcolor': colors["fill"], 'pencolor': colors["stroke"]})

        for edge in graph.edges:
            g.edge(f"{edge.start.id()}:out", f"{edge.end.id()}:in")

    plot_lg(lat_graph)

    g.save(f"{filename}.gv")
    g.render(f"{filename}.svg")

    return g


def plot_latency_graph_overview(lat_graph: lg.LatencyGraph, excl_node_patterns, input_node_patterns,
                                output_node_patterns, max_hier_level, filename):
    ##################################################
    # Compute in/out topics for hierarchy level X
    ##################################################

    def get_nodes_on_level(lat_graph: lg.LatencyGraph):
        def _traverse_node(node: lg.LGHierarchyLevel, cur_lvl=0):
            if cur_lvl == max_hier_level:
                return [node]

            if not node.children:
                return [node]

            collected_nodes = []
            for ch in node.children:
                collected_nodes += _traverse_node(ch, cur_lvl + 1)
            return collected_nodes

        return _traverse_node(lat_graph.top_node)

    lvl_nodes = get_nodes_on_level(lat_graph)
    lvl_nodes = [n for n in lvl_nodes if not any(re.search(p, n.full_name) for p in excl_node_patterns)]

    input_nodes = [n.full_name for n in lvl_nodes if any(re.search(p, n.full_name) for p in input_node_patterns)]
    output_nodes = [n.full_name for n in lvl_nodes if any(re.search(p, n.full_name) for p in output_node_patterns)]

    print(', '.join(map(lambda n: n, input_nodes)))
    print(', '.join(map(lambda n: n, output_nodes)))
    print(', '.join(map(lambda n: n.full_name, lvl_nodes)))

    def _collect_callbacks(n: lg.LGHierarchyLevel):
        callbacks = []
        callbacks += n.callbacks
        for ch in n.children:
            callbacks += _collect_callbacks(ch)
        return callbacks

    cb_to_node_map = {}
    for n in lvl_nodes:
        cbs = _collect_callbacks(n)
        for cb in cbs:
            cb_to_node_map[cb.id()] = n

    edges_between_nodes = {}
    for edge in lat_graph.edges:
        from_node = cb_to_node_map.get(edge.start.id())
        to_node = cb_to_node_map.get(edge.end.id())

        if from_node is None or to_node is None:
            continue

        if from_node.full_name == to_node.full_name:
            continue

        k = (from_node.full_name, to_node.full_name)

        if k not in edges_between_nodes:
            edges_between_nodes[k] = []

        edges_between_nodes[k].append(edge)

    g = gv.Digraph('G', filename="latency_graph.gv",
                   node_attr={'shape': 'plain'},
                   graph_attr={'pack': '1'})
    g.graph_attr['rankdir'] = 'LR'

    for n in lvl_nodes:
        colors = NODE_COLORS[NODE_NAMESPACE_MAPPING.get(n.full_name.strip("/").split("/")[0])]
        peripheries = "1" if n.full_name not in output_nodes else "2"
        g.node(n.full_name, label=n.full_name, fillcolor=colors["fill"], color=colors["stroke"],
               shape="box", style="filled", peripheries=peripheries)

        if n.full_name in input_nodes:
            helper_node_name = f"{n.full_name}__before"
            g.node(helper_node_name, label="", shape="none", height="0", width="0")
            g.edge(helper_node_name, n.full_name)

    def compute_e2e_paths(start_nodes, end_nodes, edges):
        frontier_paths = [[n] for n in start_nodes]
        final_paths = []

        while frontier_paths:
            frontier_paths_new = []

            for path in frontier_paths:
                head = path[-1]
                if head in end_nodes:
                    final_paths.append(path)
                    continue

                out_nodes = [n_to for n_from, n_to in edges if n_from == head if n_to not in path]
                new_paths = [path + [n] for n in out_nodes]
                frontier_paths_new += new_paths

            frontier_paths = frontier_paths_new

        final_paths = [[(n_from, n_to)
                        for n_from, n_to in zip(path[:-1], path[1:])]
                       for path in final_paths]
        return final_paths

    e2e_paths = compute_e2e_paths(input_nodes, output_nodes, edges_between_nodes)

    for (src_name, dst_name), edges in edges_between_nodes.items():
        print(src_name, dst_name, len(edges))
        color = "black" if any((src_name, dst_name) in path for path in e2e_paths) else "tomato"
        g.edge(src_name, dst_name, penwidth=str(math.log(len(edges)) * 2 + .2), color=color)

    g.save(f"{filename}.gv")
    g.render(f"{filename}.svg")

    return g
