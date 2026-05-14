from collections import deque
from .node import SearchNode

def bfs(initial_state):
    start_node = SearchNode(initial_state)
    if start_node.is_goal():
        return start_node.get_solution()

    frontier = deque([start_node])
    explored = {start_node}

    while frontier:
        node = frontier.popleft()
        for child in node.expand():
            if child not in explored:
                if child.is_goal():
                    return child.get_solution()
                frontier.append(child)
                explored.add(child)
    return None