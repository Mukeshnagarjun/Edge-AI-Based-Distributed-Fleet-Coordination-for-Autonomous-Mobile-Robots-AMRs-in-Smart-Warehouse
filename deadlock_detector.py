"""Utilities for detecting circular waits between AMRs."""

from collections.abc import Iterable, Mapping


def find_wait_cycles(
    wait_for_graph: Mapping[str, Iterable[str]],
) -> list[tuple[str, ...]]:
    """Return simple directed cycles in a wait-for graph.

    An edge A -> B means AMR A is waiting for AMR B.
    Each cycle is reported once, without repeating its first node.
    """
    adjacency = {
        node: set(blockers)
        for node, blockers in wait_for_graph.items()
    }

    for blockers in list(adjacency.values()):
        for blocker in blockers:
            adjacency.setdefault(blocker, set())

    ordered_nodes = sorted(adjacency)
    rank = {node: i for i, node in enumerate(ordered_nodes)}
    found = set()

    def visit(start, current, path, in_path):
        for neighbor in sorted(adjacency[current], key=rank.__getitem__):
            if neighbor == start:
                found.add(tuple(path))
                continue
            if rank[neighbor] < rank[start] or neighbor in in_path:
                continue

            path.append(neighbor)
            in_path.add(neighbor)
            visit(start, neighbor, path, in_path)
            in_path.remove(neighbor)
            path.pop()

    for start in ordered_nodes:
        visit(start, start, [start], {start})

    return sorted(
        found,
        key=lambda cycle: (len(cycle), tuple(rank[node] for node in cycle)),
    )
