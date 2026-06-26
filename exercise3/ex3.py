import ext_elev
import random
import re
import math 

id = ["213712276"]

class Controller:
    def __init__(self, game: ext_elev.GameAPI):
            self.game = game
            self.reachable = self.game.get_reachable()
            self.capacities = self.game.get_capacities()
            self.max_steps = self.game.get_max_steps()
            
            # --- Belief State (Empirical Model) ---
            self.elev_attempts = {eid: 1 for eid in self.reachable}
            self.elev_successes = {eid: 1 for eid in self.reachable}
            
            self.person_attempts = {}
            self.person_successes = {}
            self.person_rewards_sum = {}
            self.person_deliveries = {}
            
            _, persons_t, _ = self.game.get_initial_state()
            for pid, _ in persons_t:
                self.person_attempts[pid] = 1
                self.person_successes[pid] = 1
                self.person_rewards_sum[pid] = 20.0 
                self.person_deliveries[pid] = 1
                
            # --- Deterministic APSP (Transfer Stations) ---
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

            self.last_state = None
            self.last_action = None

    def choose_next_action(self, state):
            if self.last_action is not None and self.last_action != "RESET":
                self._update_beliefs(state)

            legal_actions = self._get_legal_actions(state)
            
            if not legal_actions:
                chosen = "RESET"
            else:
                # עם UCB אנחנו כבר לא צריכים חקירה אקראית נרחבת.
                # נשאיר אפסילון מינימלי של 1% רק למקרי קצה ולשבירת שוויון נדירה.
                epsilon = 0.01 
                
                if random.random() < epsilon:
                    chosen = random.choice(legal_actions)
                else:
                    chosen = max(legal_actions, key=lambda a: self._score_action(a, state))
                    
            self.last_state = state
            self.last_action = chosen
            
            return chosen

    def _score_action(self, action_str, state):
        elevators, persons, _ = state
        
        # --- טיפול בפקודת RESET וזיהוי מלכודות ה-RL ---
        if action_str == "RESET":
            alive_pids = {p for p, loc in persons}
            max_r_delivered = 0.0
            
            for p in self.person_attempts:
                if p not in alive_pids:
                    r_p = self.person_rewards_sum[p] / self.person_deliveries[p]
                    if r_p > max_r_delivered:
                        max_r_delivered = r_p
                        
            if max_r_delivered >= 35.0:
                return 100000.0 
            else:
                return -5000.0

        m = re.fullmatch(r"\s*(MOVE|ENTER|EXIT)\s*\{\s*(-?\d+)\s*,\s*(-?\d+)\s*\}\s*", action_str)
        if not m:
            return -5000.0

        name = m.group(1)
        arg1, arg2 = int(m.group(2)), int(m.group(3))
        
        STEP_PENALTY = -10.0
        curr_rem = len(persons)
        goal_r = self.game.get_goal_reward()
        
        # --- UCB: Optimism in the Face of Uncertainty ---
        # חישוב לוגריתם של הזמן הכולל ליצירת הבונוס
        t = max(1, self.game.get_current_steps())
        ln_t = math.log(t)
        
        # קבועי UCB - קובעים כמה משקל לתת לסקרנות לעומת ידע קיים
        C_PROB = 0.5 
        C_REW = 5.0
        
        # פונקציות פנימיות שמחזירות את התוחלת + הבונוס
        def get_p_p(pid):
            base_p = self.person_successes[pid] / self.person_attempts[pid]
            bonus = C_PROB * math.sqrt(ln_t / self.person_attempts[pid])
            return min(1.0, base_p + bonus) # הסתברות לא תעלה על 100%

        def get_p_e(eid):
            base_p = self.elev_successes[eid] / self.elev_attempts[eid]
            bonus = C_PROB * math.sqrt(ln_t / self.elev_attempts[eid])
            return min(1.0, base_p + bonus)

        def get_eff_reward(pid):
            base_r = self.person_rewards_sum[pid] / self.person_deliveries[pid]
            bonus = C_REW * math.sqrt(ln_t / self.person_deliveries[pid])
            eff_r = base_r + bonus
            if curr_rem == 1:
                return eff_r + goal_r
            return eff_r

        if name == "EXIT":
            pid, eid = arg1, arg2
            curr_f = next(f for e, f, w in elevators if e == eid)
            goal_f = self.game.get_person_goal(pid)
            
            p_p = get_p_p(pid)
            eff_r_p = get_eff_reward(pid)
            
            if curr_f == goal_f:
                return p_p * eff_r_p * 1000.0 
            elif curr_f in self.best_dropoffs[eid].get(goal_f, set()):
                return p_p * eff_r_p * 300.0 
            else:
                return -5000.0 

        elif name == "ENTER":
            pid, eid = arg1, arg2
            curr_f = next(loc[1] for p, loc in persons if p == pid)
            goal_f = self.game.get_person_goal(pid)
            
            p_p = get_p_p(pid)
            eff_r_p = get_eff_reward(pid)
            
            if (curr_f, goal_f) in self.helpful_pickups[eid]:
                return p_p * eff_r_p * 100.0
            else:
                return -5000.0 

        elif name == "MOVE":
            eid, target_f = arg1, arg2
            p_e = get_p_e(eid)
            
            floor_value = 0.0
            
            for pid, loc in persons:
                goal_f = self.game.get_person_goal(pid)
                eff_r_p = get_eff_reward(pid)
                
                if isinstance(loc, tuple) and loc[0] == 'in' and loc[1] == eid:
                    if target_f == goal_f:
                        floor_value += eff_r_p * 500.0
                    elif target_f in self.best_dropoffs[eid].get(goal_f, set()):
                        floor_value += eff_r_p * 200.0
                        
            for pid, loc in persons:
                goal_f = self.game.get_person_goal(pid)
                eff_r_p = get_eff_reward(pid)
                
                if isinstance(loc, tuple) and loc[0] == 'floor' and loc[1] == target_f:
                    if (target_f, goal_f) in self.helpful_pickups[eid]:
                        floor_value += eff_r_p * 50.0
                    
            return (p_e * floor_value) + STEP_PENALTY

    def _get_legal_actions(self, state):
        """פונקציית עזר ליצירת רשימת כל הפעולות החוקיות במצב הנוכחי"""
        elevators, persons, _ = state
        actions = ["RESET"]
        
        for eid, e_floor, cur_w in elevators:
            for f in self.reachable[eid]:
                if f != e_floor:
                    actions.append(f"MOVE{{{eid},{f}}}")
            
            for pid, loc in persons:
                if isinstance(loc, tuple) and loc[0] == 'floor' and loc[1] == e_floor:
                    if cur_w + self.game.get_person_weight(pid) <= self.capacities[eid]:
                        actions.append(f"ENTER{{{pid},{eid}}}")
                elif isinstance(loc, tuple) and loc[0] == 'in' and loc[1] == eid:
                    actions.append(f"EXIT{{{pid},{eid}}}")
                    
        return actions

    def _update_beliefs(self, current_state):
        """מפענח את תוצאת הפעולה הקודמת ומעדכן את המודל הסטטיסטי"""
        m = re.fullmatch(r"\s*(MOVE|ENTER|EXIT)\s*\{\s*(-?\d+)\s*,\s*(-?\d+)\s*\}\s*", self.last_action)
        if not m:
            return
            
        action_type = m.group(1)
        arg1, arg2 = int(m.group(2)), int(m.group(3))
        
        last_elevs, last_persons, last_rem = self.last_state
        curr_elevs, curr_persons, curr_rem = current_state
        
        if action_type == "MOVE":
            eid, target_f = arg1, arg2
            self.elev_attempts[eid] += 1
            curr_f = next(f for e, f, w in curr_elevs if e == eid)
            if curr_f == target_f:
                self.elev_successes[eid] += 1
                
        elif action_type == "ENTER":
            pid, eid = arg1, arg2
            self.person_attempts[pid] += 1
            curr_loc = next((loc for p, loc in curr_persons if p == pid), None)
            if isinstance(curr_loc, tuple) and curr_loc[0] == 'in' and curr_loc[1] == eid:
                self.person_successes[pid] += 1
                
        elif action_type == "EXIT":
            pid, eid = arg1, arg2
            self.person_attempts[pid] += 1
            curr_loc = next((loc for p, loc in curr_persons if p == pid), None)
            
            if isinstance(curr_loc, tuple) and curr_loc[0] == 'in' and curr_loc[1] == eid:
                pass 
            else:
                self.person_successes[pid] += 1
                if curr_loc is None: # האדם ירד ביעד ונעלם מהמפה
                    reward_gained = self.game.get_last_gained_reward()
                    if curr_rem == 0 or (last_rem == 1 and curr_rem > 1): 
                        reward_gained -= self.game.get_goal_reward()
                    self.person_rewards_sum[pid] += reward_gained
                    self.person_deliveries[pid] += 1

