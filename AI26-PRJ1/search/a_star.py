# search/a_star.py
import heapq

def _manhattan(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

class _PhaseNode:
    """گره جستجو برای هر فاز (رسیدن به یک هدف)"""
    __slots__ = ('state', 'parent', 'action', 'path_cost', 'depth',
                 'agent_pos', 'enemy_pos', '_hash_key', 'consecutive_waits')

    def __init__(self, state, parent=None, action=None, path_cost=0, depth=0, consecutive_waits=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = depth
        self.consecutive_waits = consecutive_waits

        self.agent_pos = tuple(state.get_agent_position())
        try:
            self.enemy_pos = tuple(state.get_enemy_position())
        except Exception:
            self.enemy_pos = None
            
        self._hash_key = (self.agent_pos, self.enemy_pos)

    def __eq__(self, other):
        return isinstance(other, _PhaseNode) and self._hash_key == other._hash_key

    def __hash__(self):
        return hash(self._hash_key)

    def heuristic(self, remaining_targets):
        if not remaining_targets:
            return 0.0
        return float(min(_manhattan(self.agent_pos, t) for t in remaining_targets))

    def check_goal(self, remaining_targets):
        if self.agent_pos in remaining_targets:
            return True, self.agent_pos
        return False, None

    def expand(self):
        successors = []
        added_wait = False

        try:
            enemy_next_pos = tuple(self.state.get_enemy_next_position())
        except Exception:
            enemy_next_pos = None

        for action, cost, next_state in self.state.get_successors(toward_walls=True):
            if next_state.is_collision_state():
                continue

            next_pos = tuple(next_state.get_agent_position())

            # هرس پیشگیرانه: اجتناب از خانه‌ای که دشمن در گام بعدی به آن می‌آید
            if enemy_next_pos is not None and next_pos == enemy_next_pos:
                continue

            is_wait = (next_pos == self.agent_pos)
            if is_wait:
                if added_wait:
                    continue
                added_wait = True
                if self.consecutive_waits >= 2:
                    continue
                next_waits = self.consecutive_waits + 1
            else:
                next_waits = 0

            successors.append(_PhaseNode(
                state=next_state, parent=self, action=action,
                path_cost=self.path_cost + cost, depth=self.depth + 1,
                consecutive_waits=next_waits
            ))
        return successors

    def get_solution(self):
        path, node = [], self
        while node.parent:
            path.append(node.action)
            node = node.parent
        return path[::-1]

def _a_star_single_phase(start_state, remaining_targets):
    """A* کوچک برای رسیدن از start_state به هر یک از remaining_targets."""
    start_node = _PhaseNode(start_state)

    goal_hit, collected = start_node.check_goal(remaining_targets)
    if goal_hit:
        return [], start_state, collected

    h0 = start_node.heuristic(remaining_targets)
    frontier = [(h0, h0, 0, start_node)]
    g_scores = {hash(start_node): 0}
    explored = set()
    counter = 1

    while frontier:
        f, h_val, _, node = heapq.heappop(frontier)
        node_hash = hash(node)

        if node_hash in explored:
            continue
        explored.add(node_hash)

        goal_hit, collected = node.check_goal(remaining_targets)
        if goal_hit:
            return node.get_solution(), node.state, collected

        for child in node.expand():
            child_hash = hash(child)
            new_g = child.path_cost

            goal_hit, collected = child.check_goal(remaining_targets)
            if goal_hit:
                return child.get_solution(), child.state, collected

            if child_hash not in explored and new_g < g_scores.get(child_hash, float('inf')):
                g_scores[child_hash] = new_g
                child_h = child.heuristic(remaining_targets)
                child_f = new_g + child_h
                heapq.heappush(frontier, (child_f, child_h, counter, child))
                counter += 1

    return [], None, None

def a_star(initial_state):
    """رویکرد Phased A*: تقسیم مسئله به زیرمسئله‌های کوچک‌تر."""
    all_actions = []
    current_state = initial_state

    remaining_targets = set(tuple(t) for t in current_state.get_targets_positions())

    agent_pos = tuple(current_state.get_agent_position())
    if agent_pos in remaining_targets:
        remaining_targets.discard(agent_pos)

    while remaining_targets:
        actions, final_state, collected_target = _a_star_single_phase(
            current_state, remaining_targets
        )

        if not actions and final_state is None:
            return []

        all_actions.extend(actions)
        current_state = final_state
        if collected_target is not None:
            remaining_targets.discard(collected_target)

        if not actions and collected_target is None:
            break

    return all_actions
