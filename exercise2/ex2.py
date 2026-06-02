import ext_elev
import re
import heapq
import time

# ID: 000000000
# 
# AI Usage Disclosure:
# In this assignment, I used Gemini to help model the problem as an MDP.
# Specifically, the LLM assisted in designing a Hybrid Strategy:
# 1. A Depth-2 Expectimax lookahead tree with a "Farm vs. Goal" split-heuristic to perfectly handle the _rl traps.
# 2. An embedded Real-Time A* Search for standard maps that acts deterministically 
#    but uses exact expected costs (1/P_success) derived from a Dijkstra-based APSP to aggressively complete the level.

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

        # --- זיהוי אוטומטי של מלכודות RL ---
        max_r = max(self.expected_rewards.values()) if self.expected_rewards else 0
        self.is_rl_trap = max_r > 40
        self.step_penalty = 1.15 if self.is_rl_trap else 1.0

        # 1. Deterministic APSP
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

        # 2. Stochastic APSP (Dijkstra)
        self.sp_cost = {}
        for pid in self.person_goals:
            f_goal = self.person_goals[pid]
            p_person = self.person_probs[pid]
            
            adj = { 'Goal': [] }
            for f in all_floors: adj[('floor', f)] = []
            for eid, F in self.reachable.items():
                for f in F: adj[('in', eid, f)] = []
                    
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
                        
        self.eval_cache = {}
        self.v_initial = self.evaluate_state(self.initial_state)

    def choose_next_action(self, state):
        # מנתב: במלכודות מריץ Expectimax, במפות רגילות מריץ A* מותאם הסתברויות
        if self.is_rl_trap:
            return self.run_expectimax(state)
        else:
            return self.run_astar(state)

    # =========================================================================
    # HYBRID ENGINE 1: Real-Time A* Search for Normal Maps
    # =========================================================================
    def run_astar(self, start_state):
        start_t = time.time()
        pq = []
        counter = 0
        
        # מרחיב את הקודקוד הראשון (שורש)
        for a in self.generate_pruned_actions(start_state):
            succ = self.simulate_deterministic(start_state, a)
            cost = self.get_action_expected_cost(a)
            f = cost + self.heuristic_astar(succ)
            heapq.heappush(pq, (f, counter, cost, succ, a))
            counter += 1
            
        visited = {start_state: 0.0}
        best_action = "RESET"
        best_f = float('inf')
        expansions = 0
        
        while pq:
            # מגבלת זמן בטוחה מאוד כדי לא לחטוף Timeout (הקצבנו 2 שניות לצעד)
            if expansions > 3000 or time.time() - start_t > 2.0:
                break
                
            f, _, g, s, first_a = heapq.heappop(pq)
            
            if s[2] == 0: # מצאנו את הדרך המהירה ביותר ליעד!
                return first_a
                
            if s in visited and visited[s] <= g:
                continue
            visited[s] = g
            
            if f < best_f:
                best_f = f
                best_action = first_a
                
            for a in self.generate_pruned_actions(s):
                succ = self.simulate_deterministic(s, a)
                cost = self.get_action_expected_cost(a)
                nxt_g = g + cost
                if succ not in visited or nxt_g < visited[succ]:
                    nxt_f = nxt_g + self.heuristic_astar(succ)
                    heapq.heappush(pq, (nxt_f, counter, nxt_g, succ, first_a))
                    counter += 1
            
            expansions += 1
            
        return best_action if best_action else "RESET"

    def simulate_deterministic(self, state, action_str):
        """ מריץ את הפעולה בהנחת 100% הצלחה לטובת חיפוש ה-A* """
        elevators_t, persons_t, rem = state
        m = re.fullmatch(r"\s*(MOVE|ENTER|EXIT)\s*\{\s*(-?\d+)\s*,\s*(-?\d+)\s*\}\s*", action_str)
        name = m.group(1)
        arg1, arg2 = int(m.group(2)), int(m.group(3))
        
        if name == "MOVE":
            eid, target_f = arg1, arg2
            e_idx, cur_f, cur_w = self._find_elevator(elevators_t, eid)
            succ_elevs = list(elevators_t)
            succ_elevs[e_idx] = (eid, target_f, cur_w)
            return (tuple(sorted(succ_elevs)), persons_t, rem)
            
        elif name == "ENTER":
            pid, eid = arg1, arg2
            p_idx, _ = self._find_person(persons_t, pid)
            e_idx, cur_f, cur_w = self._find_elevator(elevators_t, eid)
            succ_elevs = list(elevators_t)
            succ_persons = list(persons_t)
            succ_elevs[e_idx] = (eid, cur_f, cur_w + self.person_weights[pid])
            succ_persons[p_idx] = (pid, ('in', eid))
            return (tuple(sorted(succ_elevs)), tuple(sorted(succ_persons)), rem)
            
        elif name == "EXIT":
            pid, eid = arg1, arg2
            p_idx, _ = self._find_person(persons_t, pid)
            e_idx, cur_f, cur_w = self._find_elevator(elevators_t, eid)
            if cur_f == self.person_goals[pid]:
                new_persons = tuple(sorted((p, loc) for p, loc in persons_t if p != pid))
                succ_elevs = list(elevators_t)
                succ_elevs[e_idx] = (eid, cur_f, cur_w - self.person_weights[pid])
                return (tuple(sorted(succ_elevs)), new_persons, rem - 1)
            else:
                succ_elevs = list(elevators_t)
                succ_persons = list(persons_t)
                succ_elevs[e_idx] = (eid, cur_f, cur_w - self.person_weights[pid])
                succ_persons[p_idx] = (pid, ('floor', cur_f))
                return (tuple(sorted(succ_elevs)), tuple(sorted(succ_persons)), rem)

    def get_action_expected_cost(self, action_str):
        if "MOVE" in action_str:
            eid = int(re.search(r"\{(-?\d+),", action_str).group(1))
            return 1.0 / self.elevator_probs[eid]
        else:
            pid = int(re.search(r"\{(-?\d+),", action_str).group(1))
            return 1.0 / self.person_probs[pid]

    def heuristic_astar(self, state):
        elevators, persons, rem = state
        if rem == 0: return 0.0
        h = 0.0
        e_floors = {eid: f for eid, f, _ in elevators}
        for pid, loc in persons:
            if isinstance(loc, tuple) and loc[0] == 'in':
                eid = loc[1]
                h += self.sp_cost[pid].get(('in', eid, e_floors[eid]), float('inf'))
            else:
                f_loc = loc[1]
                best_c = float('inf')
                for eid, e_f in e_floors.items():
                    if f_loc in self.reachable[eid]:
                        move_c = 0.0 if e_f == f_loc else (1.0 / self.elevator_probs[eid])
                        enter_c = 1.0 / self.person_probs[pid]
                        journey_c = self.sp_cost[pid].get(('in', eid, f_loc), float('inf'))
                        best_c = min(best_c, move_c + enter_c + journey_c)
                h += best_c
        return h

    # =========================================================================
    # HYBRID ENGINE 2: Expectimax for RL Traps
    # =========================================================================
    def run_expectimax(self, state):
        self.eval_cache = {} 
        best_action = "RESET"
        max_ev = self.v_initial - 1.0 
        
        legal_actions = self.generate_pruned_actions(state)
        for action in legal_actions:
            ev = self.get_action_ev(state, action, depth=2)
            if ev > max_ev:
                max_ev = ev
                best_action = action
        return best_action
        
    def get_state_ev(self, state, depth):
        actions = self.generate_pruned_actions(state)
        max_ev = self.v_initial - 1.0 
        for a in actions:
            ev = self.get_action_ev(state, a, depth)
            if ev > max_ev: max_ev = ev
        return max_ev

    def get_action_ev(self, state, action_str, depth):
        m = re.fullmatch(r"\s*(MOVE|ENTER|EXIT)\s*\{\s*(-?\d+)\s*,\s*(-?\d+)\s*\}\s*", action_str)
        name = m.group(1)
        arg1, arg2 = int(m.group(2)), int(m.group(3))
        
        elevators_t, persons_t, total_remaining = state
        def value_of(s): return self.evaluate_state(s) if depth <= 1 else self.get_state_ev(s, depth - 1)

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
            
            return p_succ * v_succ + (1 - p_succ) * v_fail - 1.0

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
            
            return p_succ * v_succ + (1 - p_succ) * value_of(state) - 1.0

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
                
            return p_succ * v_succ + (1 - p_succ) * value_of(state) - 1.0

    def evaluate_state(self, state):
        if state in self.eval_cache: return self.eval_cache[state]
        
        elevators, persons, _ = state
        v_all = 0.0
        v_profitable = 0.0
        e_floors = {eid: f for eid, f, _ in elevators}

        for pid, loc in persons:
            e_reward = self.expected_rewards[pid]
            if isinstance(loc, tuple) and loc[0] == 'in':
                expected_steps = self.sp_cost[pid].get(('in', loc[1], e_floors[loc[1]]), float('inf'))
            else:
                f_loc = loc[1]
                expected_steps = 0.0 if self.person_goals[pid] == f_loc else float('inf')
                if expected_steps == float('inf'):
                    for eid, e_f in e_floors.items():
                        if f_loc in self.reachable[eid]:
                            c = (0.0 if e_f == f_loc else 1.0/self.elevator_probs[eid]) + 1.0/self.person_probs[pid] + self.sp_cost[pid].get(('in', eid, f_loc), float('inf'))
                            expected_steps = min(expected_steps, c)
                            
            if expected_steps != float('inf'):
                person_val = e_reward - (self.step_penalty * expected_steps)
                v_all += person_val
                if person_val > 0: v_profitable += person_val
            else:
                v_all -= 1000 
                
        v_all += self.goal_reward
        v = max(v_all, v_profitable) if self.is_rl_trap else v_all
        
        self.eval_cache[state] = v
        return v

    def generate_pruned_actions(self, state):
        elevators, persons, _ = state
        actions = []
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
                    if (p_loc, f_goal) in self.helpful_pickups[eid] and self.person_weights[pid] <= remaining_cap and p_loc in reachable:
                        interesting_floors.add(p_loc)
                                
            for f in interesting_floors:
                if f != e_floor: actions.append(f"MOVE{{{eid},{f}}}")

            for pid, loc in persons:
                if isinstance(loc, tuple) and loc[0] == 'floor' and loc[1] == e_floor:
                    if (e_floor, self.person_goals[pid]) in self.helpful_pickups[eid] and cur_w + self.person_weights[pid] <= self.capacities[eid]:
                        actions.append(f"ENTER{{{pid},{eid}}}")
                elif isinstance(loc, tuple) and loc[0] == 'in' and loc[1] == eid:
                    f_goal = self.person_goals[pid]
                    if e_floor in self.best_dropoffs[eid].get(f_goal, set()) or e_floor == f_goal:
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