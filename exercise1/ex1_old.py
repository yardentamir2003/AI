import ex1_check
import search
import utils

id = ["No numbers - I'm special!"]

# This assignment was completed with the assistance of gemini and claude.
# The LLMs helped me to develop an admissible h_astar heuristic that accounts for 
# essential EXIT and ENTER actions to ensure an optimal solution.
# Moreover, it helped implementing the interesting floors logic within the 
# successor function in order to prune redundant MOVE actions and improve performance.

class ElevatorsProblem(search.Problem):
    def __init__(self, initial):
        
        # Save static values inside self and not inside a state
        self.height = initial['height']
        self.elevators_static = initial['Elevators'] # {id: (f, F, w_max)}
        self.persons_static = initial['Persons']     # {id: (f_start, w, f_goal)}
        
        # Calculate transfer floors (reachable by more than one elevator)
        # Essential for paths where a person must switch elevators to reach their goal
        self.transfer_floors = set()
        all_sets = [set(data[1]) for data in self.elevators_static.values()]
        for i, s1 in enumerate(all_sets):
            for j, s2 in enumerate(all_sets):
                if i != j:
                    self.transfer_floors.update(s1 & s2)
         
        # Map which floor pairs are connected by any elevator
        self.direct_connections = set()
        for e_id, (f, F, w_max) in self.elevators_static.items():
            for f1 in F:
                for f2 in F:
                    # If an elevator can reach both f1 and f2, they are directly connected
                    self.direct_connections.add((f1, f2))

        # State contains only the variables that change during the search
        elev_list = []
        for e_id, (f, F, w_max) in initial['Elevators'].items():
            elev_list.append((e_id, f))
            
        pers_list = []
        for p_id, (f_start, w, f_goal) in initial['Persons'].items():
            pers_list.append((p_id, f_start, False)) 
            
        # Create initial state 
        # Sort lists in order to get same hash value for all state's permutations,
        # preventing the search from visiting redundant permutations   
        initial_state = (tuple(sorted(elev_list)), tuple(sorted(pers_list)))
        search.Problem.__init__(self, initial_state)
        
        """ Constructor only needs the initial state.
        Don't forget to set the goal or implement the goal test"""
        

    def successor(self, state):
        elevators, persons = state
        successors = []
        
        # Calculate current weights for all elevators (avoid redundant calculations in the main loops)
        curr_weights = {e_id: 0 for e_id, _ in elevators}
        for p_id, p_loc, is_in in persons:
            if is_in:
                curr_weights[p_loc] += self.persons_static[p_id][1]
        
        # 1. MOVE Actions
        for i, (e_id, e_floor) in enumerate(elevators):
            reachable_floors = self.elevators_static[e_id][1]
            
            # Pruning Logic - find interesting floors
            max_w = self.elevators_static[e_id][2]
            remaining_cap = max_w - curr_weights[e_id]
            
            targets_inside = set()
            needs_transfer = False
            
            for p_id, p_loc, is_in in persons:
                # If person is in this elevator
                if is_in and p_loc == e_id:
                    p_goal = self.persons_static[p_id][2]
                    if p_goal in reachable_floors:
                        # a) This elevator can take them directly to their goal
                        targets_inside.add(p_goal)
                    else:
                        # b) This elevator cannot reach their goal (must transfer)
                        needs_transfer = True

            # If anyone inside needs a transfer, all reachable transfer floors become "interesting"
            if needs_transfer:
                targets_inside.update(self.transfer_floors & set(reachable_floors))
            
            # c) Pick-up floors: Only floors where someone is waiting and fits in the elevator
            targets_waiting = set()
            for p_id, p_loc, is_in in persons:
                if not is_in and p_loc in reachable_floors:
                    if self.persons_static[p_id][1] <= remaining_cap:
                        targets_waiting.add(p_loc)

            interesting_floors = targets_inside | targets_waiting
            
            for target_floor in interesting_floors:
                if target_floor != e_floor:
                    new_elevs = list(elevators)
                    new_elevs[i] = (e_id, target_floor)
                    action = f"MOVE{{{e_id},{target_floor}}}"
                    successors.append((action, (tuple(new_elevs), persons)))
                    
        # ENTER Actions
        for i, (e_id, e_floor) in enumerate(elevators):
            max_w = self.elevators_static[e_id][2]
            curr_w = curr_weights[e_id]
            for j, (p_id, p_loc, is_in) in enumerate(persons):
                # Person can enter if they are at the same floor and outside
                if not is_in and p_loc == e_floor:
                    # Capacity check
                    if curr_w + self.persons_static[p_id][1] <= max_w:
                        new_persons = list(persons)
                        new_persons[j] = (p_id, e_id, True)
                        successors.append((f"ENTER{{{p_id},{e_id}}}", (elevators, tuple(new_persons))))
                        
        # EXIT Actions
        for i, (e_id, e_floor) in enumerate(elevators):
            for j, (p_id, p_loc, is_in) in enumerate(persons):
                # Person can exit only if they are inside currently in this elevator
                if is_in and p_loc == e_id:
                    new_persons = list(persons)
                    # Update person, location becomes current floor, is_in becomes False
                    new_persons[j] = (p_id, e_floor, False)
                    successors.append((f"EXIT{{{p_id},{e_id}}}", (elevators, tuple(new_persons))))
                    
        return successors
    
        utils.raiseNotDefined()

    def goal_test(self, state):
        """ given a state, checks if this is the goal state, compares to the created goal state returns True/False"""
        
        pers_state = state[1]
        for p_id, loc, is_in_elev in pers_state:
        # If a person is inside an elevator, it isn't a goal state
            if is_in_elev: 
                return False
        
            # Extract person's goal floor from static list
            goal_floor = self.persons_static[p_id][2]
            if loc != goal_floor:
                return False
        return True
        utils.raiseNotDefined()

    
    def h_astar(self, node):
        state = node.state
        persons = state[1]
        h = 0
        for p_id, loc, is_in in persons:
            f_goal = self.persons_static[p_id][2]
            if not is_in and loc == f_goal:
                continue
            
            if is_in:
                # Location is elevator_id. Check if this elevator reaches f_goal
                if f_goal in self.elevators_static[loc][1]:
                    h += 1 
                else:
                    h += 3 # EXIT + ENTER + EXIT
            else:
                # Location is current floor. Check if any elevator connects current floor to f_goal
                if (loc, f_goal) in self.direct_connections:
                    h += 2 # ENTER + EXIT
                else:
                    h += 4 # ENTER + EXIT + ENTER + EXIT
        return h
        
        
        utils.raiseNotDefined()


def create_elevators_problem(game):
    print("<<create_elevators_problem")
    """ Create an elevators problem, based on the description.
    game - tuple of tuples as described in pdf file"""
    return ElevatorsProblem(game)


if __name__ == '__main__':
    ex1_check.main()
