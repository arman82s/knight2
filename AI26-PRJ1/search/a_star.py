import heapq
from .node import SearchNode

def _manhattan(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

def _mst_weight(points):
    pts = list(points)
    n = len(pts)
    if n <= 1: return 0
    in_mst = [False] * n
    min_edge = [float('inf')] * n
    min_edge[0] = 0
    mst = 0
    
    for _ in range(n):
        u, best = -1, float('inf')
        for i in range(n):
            if not in_mst[i] and min_edge[i] < best:
                best, u = min_edge[i], i
        if u == -1: break
        in_mst[u] = True
        mst += best
        for v in range(n):
            if not in_mst[v]:
                d = _manhattan(pts[u], pts[v])
                if d < min_edge[v]:
                    min_edge[v] = d
    return mst

def heuristic(node):
    targets = list(node.remaining)
    if not targets: return 0.0
    agent = node.agent_pos
    
    # Admissible: Cost to nearest target + MST of all remaining targets
    min_dist = min(_manhattan(agent, t) for t in targets)
    return float(min_dist + _mst_weight(targets))

def a_star(initial_state):
    start = SearchNode(initial_state)
    if start.is_goal():
        return start.get_solution()

    h0 = heuristic(start)
    frontier = [(start.path_cost + h0, h0, 0, start)]
    
    start_key = (hash(start), start.consecutive_waits)
    g_scores = {start_key: 0}
    explored = set()
    counter = 1

    while frontier:
        f, h_val, _, node = heapq.heappop(frontier)
        state_key = (hash(node), node.consecutive_waits)

        if state_key in explored:
            continue
        explored.add(state_key)

        if node.is_goal():
            return node.get_solution()

        for child in node.expand():
            child_key = (hash(child), child.consecutive_waits)
            
            # Skip states we have already perfectly explored
            if child_key in explored:
                continue
                
            new_g = child.path_cost

            # Strict path cost pruning
            if new_g < g_scores.get(child_key, float('inf')):
                g_scores[child_key] = new_g
                child_h = heuristic(child)
                child_f = new_g + child_h
                heapq.heappush(frontier, (child_f, child_h, counter, child))
                counter += 1

    return []