import search
import utils

# This assignment was completed with the assistance of Gemini and Claude.
# The LLMs helped me to develop an admissible h_astar heuristic that accounts for 
# essential EXIT and ENTER actions to ensure an optimal solution.
# Moreover, it helped implementing the "interesting floors" logic within the 
# successor function in order to prune redundant MOVE actions and improve performance.

class ElevatorsProblem(search.Problem):
    def __init__(self, initial):
        self.height = initial['height']
        self.elevators_static = initial['Elevators'] 
        self.persons_static = initial['Persons']     
        
        # Build APSP for minimum elevator rides
        all_floors = set()
        for e_id, (f, F, w_max) in self.elevators_static.items():
            all_floors.add(f)
            all_floors.update(F)
        
        # Build directed graph of rides (edges)
        edges = {f: set() for f in all_floors}
        for e_id, (f, F, w_max) in self.elevators_static.items():
            pickups = set(F) | {f}
            dropoffs = set(F)
            for p in pickups:
                for d in dropoffs:
                    if p != d:
                        edges[p].add(d)

        # BFS to find the minimum number of elevator rides between any two floors
        self.min_elevators = {f: {f: 0} for f in all_floors}
        for start_floor in all_floors:
            queue = [(start_floor, 0)]
            visited = {start_floor}
            while queue:
                curr, dist = queue.pop(0)
                for nxt in edges[curr]:
                    if nxt not in visited:
                        visited.add(nxt)
                        self.min_elevators[start_floor][nxt] = dist + 1
                        queue.append((nxt, dist + 1))

        # Compute helpful pickups and dropoffs for future use
        self.helpful_pickups = {e_id: set() for e_id in self.elevators_static}
        self.best_dropoffs = {e_id: {} for e_id in self.elevators_static}

        for e_id, (f, F, w_max) in self.elevators_static.items():
            pickups = set(F) | {f}
            dropoffs = set(F)
            
            for p_loc in all_floors:
                for p_goal in all_floors:
                    if p_loc == p_goal: continue
                    if p_loc not in pickups: continue
                    if p_goal not in self.min_elevators.get(p_loc, {}): continue
                    
                    rides_from_start = self.min_elevators[p_loc][p_goal]
                    
                    valid_dropoffs = set()
                    for drop_f in dropoffs:
                        if p_goal in self.min_elevators.get(drop_f, {}):
                            rides_from_drop = self.min_elevators[drop_f][p_goal]
                            # Dropoff is valid only if it reduces the remaining rides
                            if rides_from_drop < rides_from_start:
                                valid_dropoffs.add(drop_f)
                                
                    if valid_dropoffs:
                        self.helpful_pickups[e_id].add((p_loc, p_goal))
                        if p_goal not in self.best_dropoffs[e_id]:
                            self.best_dropoffs[e_id][p_goal] = set()
                        self.best_dropoffs[e_id][p_goal].update(valid_dropoffs)

        # Initialize state variables
        elev_list = []
        for e_id, (f, F, w_max) in self.elevators_static.items():
            elev_list.append((e_id, f))
            
        pers_list = []
        for p_id, (f_start, w, f_goal) in self.persons_static.items():
            pers_list.append((p_id, f_start, False)) 
            
        initial_state = (tuple(sorted(elev_list)), tuple(sorted(pers_list)))
        search.Problem.__init__(self, initial_state)

    def successor(self, state):
        elevators, persons = state
        successors = []
        
        curr_weights = {e_id: 0 for e_id, _ in elevators}
        for p_id, p_loc, is_in in persons:
            if is_in:
                curr_weights[p_loc] += self.persons_static[p_id][1]
        
        # MOVE Actions
        for i, (e_id, e_floor) in enumerate(elevators):
            reachable_floors = self.elevators_static[e_id][1]
            max_w = self.elevators_static[e_id][2]
            remaining_cap = max_w - curr_weights[e_id]
            
            interesting_floors = set()
            
            # Target drops for people currently inside an elevator
            for p_id, p_loc, is_in in persons:
                if is_in and p_loc == e_id:
                    p_goal = self.persons_static[p_id][2]
                    valid_drops = self.best_dropoffs[e_id].get(p_goal, set())
                    for d in valid_drops:
                        if d in reachable_floors:
                            interesting_floors.add(d)

            # Target pickups for people currently waiting
            for p_id, p_loc, is_in in persons:
                if not is_in:
                    p_goal = self.persons_static[p_id][2]
                    # Only move to pick them up if this elevator actually helps them
                    if (p_loc, p_goal) in self.helpful_pickups[e_id]:
                        if self.persons_static[p_id][1] <= remaining_cap:
                            if p_loc in reachable_floors:
                                interesting_floors.add(p_loc)

            for target_floor in interesting_floors:
                if target_floor != e_floor:
                    new_elevs = list(elevators)
                    new_elevs[i] = (e_id, target_floor)
                    action = f"MOVE{{{e_id},{target_floor}}}"
                    successors.append((action, (tuple(sorted(new_elevs)), persons)))
                    
        # ENTER Actions
        for i, (e_id, e_floor) in enumerate(elevators):
            max_w = self.elevators_static[e_id][2]
            curr_w = curr_weights[e_id]
            for j, (p_id, p_loc, is_in) in enumerate(persons):
                if not is_in and p_loc == e_floor:
                    p_goal = self.persons_static[p_id][2]
                    # Only allow them to enter if this elevator is helpful
                    if (p_loc, p_goal) in self.helpful_pickups[e_id]:
                        if curr_w + self.persons_static[p_id][1] <= max_w:
                            new_persons = list(persons)
                            new_persons[j] = (p_id, e_id, True)
                            successors.append((f"ENTER{{{p_id},{e_id}}}", (elevators, tuple(sorted(new_persons)))))
                        
        # EXIT Actions
        for i, (e_id, e_floor) in enumerate(elevators):
            for j, (p_id, p_loc, is_in) in enumerate(persons):
                if is_in and p_loc == e_id:
                    p_goal = self.persons_static[p_id][2]
                    valid_drops = self.best_dropoffs[e_id].get(p_goal, set())
                    # Only allow exit if it's the optimal dropoff or the final goal
                    if e_floor in valid_drops or e_floor == p_goal:
                        new_persons = list(persons)
                        new_persons[j] = (p_id, e_floor, False)
                        successors.append((f"EXIT{{{p_id},{e_id}}}", (elevators, tuple(sorted(new_persons)))))
                    
        return successors

    def goal_test(self, state):
        for p_id, loc, is_in_elev in state[1]:
            if is_in_elev or loc != self.persons_static[p_id][2]: 
                return False
        return True
    
    def h_astar(self, node):
        state = node.state
        persons = state[1]
        h = 0
        for p_id, loc, is_in in persons:
            f_goal = self.persons_static[p_id][2]
            if not is_in and loc == f_goal:
                continue
            
            if is_in:
                # Inside elevator, minimum cost is 1 EXIT + 2 * remaining_rides
                valid_drops = self.best_dropoffs[loc].get(f_goal, set())
                if valid_drops:
                    min_remaining = min(self.min_elevators.get(d, {}).get(f_goal, float('inf')) for d in valid_drops)
                    h += 1 + (2 * min_remaining)
                else:
                    h += 1
            else:
                # Outside elevator, minimum cost is 2 * required_rides
                if f_goal in self.min_elevators.get(loc, {}):
                    h += 2 * self.min_elevators[loc][f_goal]
        return h

def create_elevators_problem(game):
    return ElevatorsProblem(game)