import heapq
from .node import SearchNode

def ucs(initial_state):
    start_node = SearchNode(initial_state)
    if start_node.is_goal():
        return start_node.get_solution()

    frontier = [(start_node.path_cost, id(start_node), start_node)]
    explored = {}

    while frontier:
        cost, _, node = heapq.heappop(frontier)

        if node.is_goal():
            return node.get_solution()

        state_hash = hash(node)
        if state_hash in explored and explored[state_hash] <= cost:
            continue
        explored[state_hash] = cost

        for child in node.expand():
            child_hash = hash(child)
            if child_hash not in explored or explored[child_hash] > child.path_cost:
                heapq.heappush(frontier, (child.path_cost, id(child), child))
    return None