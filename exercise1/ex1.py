import ex1_check
# import TA.AI.Semester_B.Assignments.Assignment1.publish_files.search as search
# import TA.AI.Semester_B.Assignments.Assignment1.publish_files.utils as utils
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
        initial_state = (tuple(sorted(elev_list)), tuple(sorted(pers_list)))
        search.Problem.__init__(self, initial_state)
        
        """ Constructor only needs the initial state.
        Don't forget to set the goal or implement the goal test"""
        

    def successor(self, state):
        elevators, persons = state
        successors = []
        
        # 3. חישוב משקלים של כל המעליות בבת אחת (O(P) במקום O(E*P))
        curr_weights = {e_id: 0 for e_id, _ in elevators}
        for p_id, p_loc, is_in in persons:
            if is_in:
                curr_weights[p_loc] += self.persons_static[p_id][1]

        # 1. MOVE Actions
        for i, (e_id, e_floor) in enumerate(elevators):
            reachable_floors = self.elevators_static[e_id][1]
            
            # קומות "מעניינות" באמת
            targets_inside = {self.persons_static[p[0]][2] for p in persons if p[2] and p[1] == e_id and self.persons_static[p[0]][2] in reachable_floors}
            needs_transfer = any(p[2] and p[1] == e_id and self.persons_static[p[0]][2] not in reachable_floors for p in persons)
            targets_waiting = {p[1] for p in persons if not p[2] and p[1] in reachable_floors}

            interesting_floors = targets_inside | targets_waiting
            if needs_transfer:
                interesting_floors.update(self.transfer_floors & set(reachable_floors))

            for target_floor in interesting_floors:
                if target_floor != e_floor:
                    new_elevs = list(elevators)
                    new_elevs[i] = (e_id, target_floor)
                    action = f"MOVE{{{e_id},{target_floor}}}"
                    successors.append((action, (tuple(new_elevs), persons)))
                    
        # 2. ENTER Actions
        for i, (e_id, e_floor) in enumerate(elevators):
            max_w = self.elevators_static[e_id][2]
            curr_w = curr_weights[e_id]
            for j, (p_id, p_loc, is_in) in enumerate(persons):
                if not is_in and p_loc == e_floor:
                    if curr_w + self.persons_static[p_id][1] <= max_w:
                        new_persons = list(persons)
                        new_persons[j] = (p_id, e_id, True)
                        successors.append((f"ENTER{{{p_id},{e_id}}}", (elevators, tuple(new_persons))))
                        
        # 3. EXIT Actions
        for i, (e_id, e_floor) in enumerate(elevators):
            for j, (p_id, p_loc, is_in) in enumerate(persons):
                if is_in and p_loc == e_id:
                    new_persons = list(persons)
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
                # האם המעלית הנוכחית מגיעה ליעד?
                h += 1 if f_goal in self.elevators_static[loc][1] else 3
            else:
                # שימוש בחישוב המוקדם מה-init
                h += 2 if self.can_reach_goal_direct[p_id] else 4
        return h
        utils.raiseNotDefined()


def create_elevators_problem(game):
    print("<<create_elevators_problem")
    """ Create an elevators problem, based on the description.
    game - tuple of tuples as described in pdf file"""
    return ElevatorsProblem(game)


if __name__ == '__main__':
    ex1_check.main()
