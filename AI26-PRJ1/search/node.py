class SearchNode:
    def __init__(self, state, parent=None, action=None, path_cost=0, depth=0, collected=None, consecutive_waits=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = depth
        self.agent_pos = tuple(state.get_agent_position())
        
        if parent is None:
            self.all_targets = frozenset(tuple(t) for t in state.get_targets_positions())
            self.collected = frozenset()
        else:
            self.all_targets = parent.all_targets
            self.collected = parent.collected if collected is None else collected

        if self.agent_pos in self.all_targets and self.agent_pos not in self.collected:
            self.collected = self.collected | {self.agent_pos}

        self.remaining = self.all_targets - self.collected
        
        # STRICT LOGICAL HASH: Agent pos and collected targets ONLY.
        self._hash_key = (self.agent_pos, self.collected)
        self.consecutive_waits = consecutive_waits

    def __eq__(self, other):
        return isinstance(other, SearchNode) and self._hash_key == other._hash_key

    def __hash__(self):
        return hash(self._hash_key)

    def is_goal(self):
        return len(self.remaining) == 0

    def expand(self):
        successors = []
        added_wait = False
        
        # Safely get enemy distance to intelligently prune useless waits
        raw_enemy_pos = self.state.get_enemy_position()
        if raw_enemy_pos is not None:
            enemy_pos = tuple(raw_enemy_pos)
            dist_to_enemy = abs(self.agent_pos[0] - enemy_pos[0]) + abs(self.agent_pos[1] - enemy_pos[1])
        else:
            dist_to_enemy = float('inf')

        for action, cost, next_state in self.state.get_successors(toward_walls=True):
            if next_state.is_collision_state():
                continue

            next_agent_pos = tuple(next_state.get_agent_position())
            is_wait = (next_agent_pos == self.agent_pos)

            if is_wait:
                # 1. Deduplicate: Wall-bumping in corners yields multiple identical waits
                if added_wait:
                    continue
                added_wait = True

                # 2. Prune Useless Dodges: Do not evaluate waits if the enemy is far.
                # This eliminates the 3x state multiplier on 95% of the map.
                if dist_to_enemy > 3:
                    continue
                
                # 3. Cap consecutive waits to prevent infinite stalling
                next_waits = self.consecutive_waits + 1
                if next_waits > 2:
                    continue
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