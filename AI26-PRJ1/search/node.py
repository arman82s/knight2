# search/node.py
class SearchNode:
    def __init__(self, state, parent=None, action=None, path_cost=0, depth=0, collected=None, consecutive_waits=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = depth
        self.agent_pos = tuple(state.get_agent_position())
        self.consecutive_waits = consecutive_waits

        if parent is None:
            self.all_targets = frozenset(tuple(t) for t in state.get_targets_positions())
            self.collected = frozenset()
        else:
            self.all_targets = parent.all_targets
            self.collected = parent.collected if collected is None else collected

        if self.agent_pos in self.all_targets and self.agent_pos not in self.collected:
            self.collected = self.collected | {self.agent_pos}

        self.remaining = self.all_targets - self.collected
        self._hash_key = (self.agent_pos, self.collected)

    def __eq__(self, other):
        return isinstance(other, SearchNode) and self._hash_key == other._hash_key

    def __hash__(self):
        return hash(self._hash_key)

    def is_goal(self):
        return self.state.is_goal_state()

    def expand(self):
        successors = []
        added_wait = False
        for action, cost, next_state in self.state.get_successors(toward_walls=True):
            if next_state.is_collision_state():
                continue
            next_pos = tuple(next_state.get_agent_position())
            is_wait = (next_pos == self.agent_pos)
            if is_wait:
                if added_wait: continue
                added_wait = True
                if self.consecutive_waits >= 2: continue
                next_waits = self.consecutive_waits + 1
            else:
                next_waits = 0
            successors.append(SearchNode(
                state=next_state, parent=self, action=action,
                path_cost=self.path_cost + cost, depth=self.depth + 1,
                collected=self.collected, consecutive_waits=next_waits
            ))
        return successors

    def get_solution(self):
        path, node = [], self
        while node.parent:
            path.append(node.action)
            node = node.parent
        return path[::-1]
