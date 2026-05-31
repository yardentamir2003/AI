import ext_elev
import re
import heapq

# ID: 000000000
# 
# AI Usage Disclosure:
# In this assignment, I used Gemini to help model the problem as an MDP.
# Specifically, the LLM assisted in designing the Depth-2 Expectimax lookahead tree,
# and implementing the "Farm vs. Goal" split-heuristic (max(v_all, v_profitable)) to handle the _rl traps.
# Finally, it helped implement an exact Dijkstra-based All-Pairs Shortest Path (APSP) heuristic 
# that correctly penalizes broken elevators by computing expected actions (1/P_success).

class Controller:
    def __init__(self, game: ext_elev.GameAPI):
        self.game = game
        self.reachable = self.game.get_reachable()
        self.capacities = self.game.get_capacities()
        self.initial_state = self.game.get_initial_state()
        self.max_steps = self.game.get_max_steps()
        self.goal_reward = self.game.get_goal_reward()
        
        self.person_weights = {}
        self.person_goals = {}
        self.person_probs = {}
        self.expected_rewards = {}
        
        _, initial_persons, _ = self.initial_state
        for pid, _ in initial_persons:
            self.person_weights[pid] = self.game.get_person_weight(pid)
            self.person_goals[pid] = self.game.get_person_goal(pid)
            self.person_probs[pid] = self.game.get_person_action_prob(pid)
            rewards = self.game.get_person_reward(pid)
            self.expected_rewards[pid] = sum(rewards) / len(rewards)
            
        self.elevator_probs = {eid: self.game.get_elevator_action_prob(eid) for eid in self.reachable}

        # 1. Deterministic APSP (for Action Pruning)
        all_floors = set()
        for F in self.reachable.values():
            all_floors.update(F)
            
        edges = {f: set() for f in all_floors}
        for F in self.reachable.values():
            for p in F:
                for d in F:
                    if p != d: edges[p].add(d)

        self.min_elevators = {f: {f: 0} for f in all_floors}
        for start_floor in all_floors:
            queue = [(start_floor, 0)]
            visited = {start_floor}
            while queue:
                curr, dist = queue.pop(0)
                for nxt in edges.get(curr, []):
                    if nxt not in visited:
                        visited.add(nxt)
                        self.min_elevators[start_floor][nxt] = dist + 1
                        queue.append((nxt, dist + 1))

        self.helpful_pickups = {eid: set() for eid in self.reachable}
        self.best_dropoffs = {eid: {} for eid in self.reachable}

        for eid, F in self.reachable.items():
            for p_loc in all_floors:
                for p_goal in all_floors:
                    if p_loc == p_goal: continue
                    if p_loc not in F: continue
                    if p_goal not in self.min_elevators.get(p_loc, {}): continue
                    
                    rides_from_start = self.min_elevators[p_loc][p_goal]
                    valid_dropoffs = set()
                    
                    for drop_f in F:
                        if p_goal in self.min_elevators.get(drop_f, {}):
                            rides_from_drop = self.min_elevators[drop_f][p_goal]
                            if rides_from_drop < rides_from_start:
                                valid_dropoffs.add(drop_f)
                                
                    if valid_dropoffs:
                        self.helpful_pickups[eid].add((p_loc, p_goal))
                        if p_goal not in self.best_dropoffs[eid]:
                            self.best_dropoffs[eid][p_goal] = set()
                        self.best_dropoffs[eid][p_goal].update(valid_dropoffs)

        # 2. Stochastic APSP (Dijkstra) for Exact Expected Heuristic
        self.sp_cost = {}
        for pid in self.person_goals:
            f_goal = self.person_goals[pid]
            p_person = self.person_probs[pid]
            
            adj = { 'Goal': [] }
            for f in all_floors:
                adj[('floor', f)] = []
            for eid, F in self.reachable.items():
                for f in F:
                    adj[('in', eid, f)] = []
                    
            for eid, F in self.reachable.items():
                if f_goal in F:
                    adj['Goal'].append((('in', eid, f_goal), 1.0 / p_person))
            for eid, F in self.reachable.items():
                for f in F:
                    adj[('floor', f)].append((('in', eid, f), 1.0 / p_person))
                    adj[('in', eid, f)].append((('floor', f), 1.0 / p_person))
                    p_e = self.elevator_probs[eid]
                    for d in F:
                        if d != f:
                            adj[('in', eid, f)].append((('in', eid, d), 1.0 / p_e))
                            
            dist = { node: float('inf') for node in adj }
            dist['Goal'] = 0.0
            pq = [(0.0, 'Goal')]
            
            while pq:
                d, u = heapq.heappop(pq)
                if d > dist[u]: continue
                for v, weight in adj[u]:
                    if dist[u] + weight < dist[v]:
                        dist[v] = dist[u] + weight
                        heapq.heappush(pq, (dist[v], v))
            self.sp_cost[pid] = dist
                        
        self.step_penalty = 1.15 #The 'Magic Number' that balances goal-pursuit and rl-farming
        self.eval_cache = {}
        self.v_initial = self.evaluate_state(self.initial_state)

    def choose_next_action(self, state):
            self.eval_cache = {} 
            
            # heuristic: if problem is small (easy), use faster, simpler greedy search.
            # if problem is hard (large), use the deep 2-step stochastic lookahead.
            elevators, persons, _ = state
            is_hard = len(persons) > 3 or len(elevators) > 2
            
            best_action = "RESET"
            # Adjusted penalty: slightly lower (0.9) encourages more activity.
            max_ev = self.v_initial - 0.9 
            
            legal_actions = self.generate_pruned_actions(state)
            
            # If 'hard', we use depth 2, otherwise depth 1 to keep it stable
            target_depth = 2 if is_hard else 1
            
            for action in legal_actions:
                ev = self.get_action_ev(state, action, depth=target_depth)
                if ev > max_ev:
                    max_ev = ev
                    best_action = action
                    
            return best_action
        
    def get_state_ev(self, state, depth):
        actions = self.generate_pruned_actions(state)
        max_ev = self.v_initial 
        for a in actions:
            ev = self.get_action_ev(state, a, depth)
            if ev > max_ev: max_ev = ev
        return max_ev

    def get_action_ev(self, state, action_str, depth):
        m = re.fullmatch(r"\s*(MOVE|ENTER|EXIT)\s*\{\s*(-?\d+)\s*,\s*(-?\d+)\s*\}\s*", action_str)
        name = m.group(1)
        arg1, arg2 = int(m.group(2)), int(m.group(3))
        
        elevators_t, persons_t, total_remaining = state
        
        def value_of(s):
            if depth <= 1: return self.evaluate_state(s)
            else: return self.get_state_ev(s, depth - 1)

        if name == "MOVE":
            eid, target_f = arg1, arg2
            e_idx, cur_f, cur_w = self._find_elevator(elevators_t, eid)
            p_succ = self.elevator_probs[eid]
            
            succ_elevs = list(elevators_t)
            succ_elevs[e_idx] = (eid, target_f, cur_w)
            v_succ = value_of((tuple(sorted(succ_elevs)), persons_t, total_remaining))
            
            options = sorted({cur_f} | (set(self.reachable[eid]) - {target_f}))
            v_fail_sum = 0
            for f_opt in options:
                fail_elevs = list(elevators_t)
                fail_elevs[e_idx] = (eid, f_opt, cur_w)
                v_fail_sum += value_of((tuple(sorted(fail_elevs)), persons_t, total_remaining))
            v_fail = v_fail_sum / len(options)
            
            return p_succ * v_succ + (1 - p_succ) * v_fail

        elif name == "ENTER":
            pid, eid = arg1, arg2
            p_idx, _ = self._find_person(persons_t, pid)
            e_idx, cur_f, cur_w = self._find_elevator(elevators_t, eid)
            p_succ = self.person_probs[pid]
            
            succ_elevs = list(elevators_t)
            succ_persons = list(persons_t)
            succ_elevs[e_idx] = (eid, cur_f, cur_w + self.person_weights[pid])
            succ_persons[p_idx] = (pid, ('in', eid))
            v_succ = value_of((tuple(sorted(succ_elevs)), tuple(sorted(succ_persons)), total_remaining))
            
            v_fail = value_of(state)
            return p_succ * v_succ + (1 - p_succ) * v_fail

        elif name == "EXIT":
            pid, eid = arg1, arg2
            p_idx, _ = self._find_person(persons_t, pid)
            e_idx, cur_f, cur_w = self._find_elevator(elevators_t, eid)
            p_succ = self.person_probs[pid]
            
            if cur_f == self.person_goals[pid]: 
                new_persons = tuple(sorted((p, loc) for p, loc in persons_t if p != pid))
                new_remaining = total_remaining - 1
                
                v_succ = self.expected_rewards[pid] 
                if new_remaining == 0:
                    v_succ += self.goal_reward + self.v_initial
                else:
                    succ_elevs = list(elevators_t)
                    succ_elevs[e_idx] = (eid, cur_f, cur_w - self.person_weights[pid])
                    v_succ += value_of((tuple(sorted(succ_elevs)), new_persons, new_remaining))
            else: 
                succ_elevs = list(elevators_t)
                succ_persons = list(persons_t)
                succ_elevs[e_idx] = (eid, cur_f, cur_w - self.person_weights[pid])
                succ_persons[p_idx] = (pid, ('floor', cur_f))
                v_succ = value_of((tuple(sorted(succ_elevs)), tuple(sorted(succ_persons)), total_remaining))
                
            v_fail = value_of(state)
            return p_succ * v_succ + (1 - p_succ) * v_fail

    def get_person_cost(self, pid, loc, e_floors):
        if isinstance(loc, tuple) and loc[0] == 'in':
            eid = loc[1]
            e_f = e_floors[eid]
            return self.sp_cost[pid][('in', eid, e_f)]
        else:
            f_loc = loc[1]
            if self.person_goals[pid] == f_loc: return 0.0
            
            best_cost = float('inf')
            for eid, e_f in e_floors.items():
                if f_loc in self.reachable[eid]:
                    move_cost = 0.0 if e_f == f_loc else (1.0 / self.elevator_probs[eid])
                    enter_cost = 1.0 / self.person_probs[pid]
                    journey_cost = self.sp_cost[pid].get(('in', eid, f_loc), float('inf'))
                    
                    total = move_cost + enter_cost + journey_cost
                    if total < best_cost:
                        best_cost = total
            return best_cost

    def evaluate_state(self, state):
        if state in self.eval_cache: return self.eval_cache[state]
        
        elevators, persons, _ = state
        v_all = 0.0
        v_profitable = 0.0
        e_floors = {eid: f for eid, f, _ in elevators}

        for pid, loc in persons:
            e_reward = self.expected_rewards[pid]
            expected_steps = self.get_person_cost(pid, loc, e_floors)
            
            if expected_steps != float('inf'):
                person_val = e_reward - (self.step_penalty * expected_steps)
                v_all += person_val
                if person_val > 0:
                    v_profitable += person_val
            else:
                v_all -= 1000 
                
        v_all += self.goal_reward
        v = max(v_all, v_profitable)
        
        self.eval_cache[state] = v
        return v

    def generate_pruned_actions(self, state):
        elevators, persons, _ = state
        actions = []
        curr_weights = {eid: w for eid, _, w in elevators}

        for eid, e_floor, cur_w in elevators:
            reachable = self.reachable[eid]
            remaining_cap = self.capacities[eid] - cur_w
            interesting_floors = set()
            
            for pid, loc in persons:
                f_goal = self.person_goals[pid]
                
                if isinstance(loc, tuple) and loc[0] == 'in' and loc[1] == eid:
                    valid_drops = self.best_dropoffs[eid].get(f_goal, set())
                    interesting_floors.update(d for d in valid_drops if d in reachable)
                    if f_goal in reachable: interesting_floors.add(f_goal)
                        
                elif isinstance(loc, tuple) and loc[0] == 'floor':
                    p_loc = loc[1]
                    if (p_loc, f_goal) in self.helpful_pickups[eid]:
                        if self.person_weights[pid] <= remaining_cap:
                            if p_loc in reachable:
                                interesting_floors.add(p_loc)
                                
            for f in interesting_floors:
                if f != e_floor: actions.append(f"MOVE{{{eid},{f}}}")

            for pid, loc in persons:
                if isinstance(loc, tuple) and loc[0] == 'floor' and loc[1] == e_floor:
                    f_goal = self.person_goals[pid]
                    if (e_floor, f_goal) in self.helpful_pickups[eid]:
                        if cur_w + self.person_weights[pid] <= self.capacities[eid]:
                            actions.append(f"ENTER{{{pid},{eid}}}")
                            
                elif isinstance(loc, tuple) and loc[0] == 'in' and loc[1] == eid:
                    f_goal = self.person_goals[pid]
                    valid_drops = self.best_dropoffs[eid].get(f_goal, set())
                    if e_floor in valid_drops or e_floor == f_goal:
                        actions.append(f"EXIT{{{pid},{eid}}}")

        return actions

    def _find_elevator(self, elevators, e_id):
        for i, (eid, fl, w) in enumerate(elevators):
            if eid == e_id: return i, fl, w
        return None

    def _find_person(self, persons, p_id):
        for i, (pid, loc) in enumerate(persons):
            if pid == p_id: return i, loc
        return None