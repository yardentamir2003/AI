import ex1_check
import search
import utils

id = ["No numbers - I'm special!"]



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

        # Calculate direct reachability for each person
        # Check if any single elevator can take a person from their start floor directly to goal floor
        self.can_reach_goal_direct = {}
        for p_id, (f_start, w, f_goal) in self.persons_static.items():
            # Check if there exists an elevator where both start and goal floors are in its reachable set (F)
            self.can_reach_goal_direct[p_id] = any(
                f_start in elev_data[1] and f_goal in elev_data[1] 
                for elev_data in self.elevators_static.values()
            )

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

        # # MOVE Actions
        # for i, (e_id, e_floor) in enumerate(elevators):
        #     reachable_floors = self.elevators_static[e_id][1]
            
        #     # Identify "interesting floors" in order to prune useless moves
            
        #     # a) Destinations of people currently inside this elevator
        #     targets_inside = {self.persons_static[p[0]][2] for p in persons if p[2] and p[1] == e_id and self.persons_static[p[0]][2] in reachable_floors}
        #     # b) Floors where people are waiting to be picked up            
        #     targets_waiting = {p[1] for p in persons if not p[2] and p[1] in reachable_floors}
        #     # c) Check if someone inside must switch elevators
        #     needs_transfer = any(p[2] and p[1] == e_id and self.persons_static[p[0]][2] not in reachable_floors for p in persons)
           
        #     # If a transfer is needed, add pre-calculated meeting points
        #     interesting_floors = targets_inside | targets_waiting
        #     if needs_transfer:
        #         interesting_floors.update(self.transfer_floors & set(reachable_floors))

        #     # Generate move actions only to "interesting destinations"
        #     for target_floor in interesting_floors:
        #         if target_floor != e_floor:
        #             new_elevs = list(elevators)
        #             new_elevs[i] = (e_id, target_floor)
        #             action = f"MOVE{{{e_id},{target_floor}}}"
        #             successors.append((action, (tuple(new_elevs), persons)))
        
        
        # 1. MOVE Actions (with pruning)
        for i, (e_id, e_floor) in enumerate(elevators):
            # Identify "interesting floors" in order to prune useless moves
            reachable_floors = self.elevators_static[e_id][1]
            max_w = self.elevators_static[e_id][2]
            curr_w = curr_weights[e_id]
            remaining_cap = max_w - curr_w

            # a) Destinations of people currently inside this elevator
            targets_inside = {self.persons_static[p[0]][2] for p in persons 
                              if p[2] and p[1] == e_id and self.persons_static[p[0]][2] in reachable_floors}
            
            # b) Floors where people are waiting AND can actually fit in the elevator
            targets_waiting = set()
            for p_id, p_loc, is_in in persons:
                if not is_in and p_loc in reachable_floors:
                    # Check if this specific person's weight fits in the remaining capacity
                    if self.persons_static[p_id][1] <= remaining_cap:
                        targets_waiting.add(p_loc)

            # c) Check if someone inside must switch elevators
            needs_transfer = any(p[2] and p[1] == e_id and self.persons_static[p[0]][2] not in reachable_floors 
                                 for p in persons)
            
            # Combine all interesting floors
            interesting_floors = targets_inside | targets_waiting
            if needs_transfer:
                interesting_floors.update(self.transfer_floors & set(reachable_floors))

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
        # A* Estimates the minimum number of actions required for all people to reach their destinations
        # The estimate is based on the minimum required ENTER and EXIT actions
        state = node.state
        persons = state[1]
        h = 0
        
        for p_id, loc, is_in in persons:
            f_goal = self.persons_static[p_id][2]
            
            # Case 0: Person is already at their goal floor and outside the elevator, Cost = 0
            if not is_in and loc == f_goal:
                continue
                
            if is_in:
                # Case 1: Person is inside an elevator. Check if this elevator can reach person's goal floor
                if f_goal in self.elevators_static[loc][1]:
                    # Minimum 1 action required: EXIT at the goal floor
                    h += 1 
                else:
                    # Minimum 3 actions required: EXIT + ENTER + EXIT
                    h += 3 
            else:
                # Case 2: Person is outside an elevator. Check if any elevator can take them directly to goal
                if self.can_reach_goal_direct[p_id]:
                    # Minimum 2 actions required: ENTER + EXIT
                    h += 2 
                else:
                    # Minimum 4 actions required: ENTER + EXIT + ENTER + EXIT
                    h += 4
                    
        return h
        
        
        utils.raiseNotDefined()


def create_elevators_problem(game):
    print("<<create_elevators_problem")
    """ Create an elevators problem, based on the description.
    game - tuple of tuples as described in pdf file"""
    return ElevatorsProblem(game)


if __name__ == '__main__':
    ex1_check.main()
