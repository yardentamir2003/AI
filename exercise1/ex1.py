import ex1_check
# import TA.AI.Semester_B.Assignments.Assignment1.publish_files.search as search
# import TA.AI.Semester_B.Assignments.Assignment1.publish_files.utils as utils
import search
import utils

id = ["No numbers - I'm special!"]



class ElevatorsProblem(search.Problem):
    """This class implements an elevators problem"""

    def __init__(self, initial):
        # Save static values inside self and not inside a state
        self.height = initial['height']
        self.elevators_static = initial['Elevators'] # {id: (f, F, w_max)}
        self.persons_static = initial['Persons']     # {id: (f_start, w, f_goal)}
        
        # Create initial state (tuple of tuples)
        elev_list = []
        for e_id, (f, F, w_max) in initial['Elevators'].items():
            elev_list.append((e_id, f)) # (e_id, current_floor)
            
        pers_list = []
        for p_id, (f_start, w, f_goal) in initial['Persons'].items():
            # Person starts at his starting floor, outside the elevator
            pers_list.append((p_id, f_start, False)) # (p_id, location(floor\e_id), is_in_elevator) 
            
        # Sort lists in order to get same hash value for all state's permutations
        initial_state = (tuple(sorted(elev_list)), tuple(sorted(pers_list)))
        search.Problem.__init__(self, initial_state)
        
        """ Constructor only needs the initial state.
        Don't forget to set the goal or implement the goal test"""
        

    def successor(self, state):
        """ Generates the successor states returns [(action, achieved_states, ...)]"""
        
        # Extract elevators and persons from the state 
        elevators, persons = state
        successors = []
        
        # # 1. MOVE Actions
        # for i, (e_id, e_floor) in enumerate(elevators):
        #     reachable_floors = self.elevators_static[e_id][1] # extract F (reachable floors) from static data
        #     for target_floor in reachable_floors:
        #         if target_floor != e_floor:
        #             new_elevs = list(elevators)
        #             new_elevs[i] = (e_id, target_floor)
        #             action = f"MOVE{{{e_id},{target_floor}}}"
        #             successors.append((action, (tuple(new_elevs), persons)))
        
        # 1. MOVE Actions - Smart Pruning
        for i, (e_id, e_floor) in enumerate(elevators):
            reachable_floors = self.elevators_static[e_id][1]
            
            # קומות "מעניינות" למעלית הזו:
            # א. קומות יעד של אנשים שנמצאים כרגע בתוך המעלית הזו
            targets_inside = {self.persons_static[p[0]][2] for p in persons if p[2] and p[1] == e_id}
            
            # ב. קומות שבהן מחכים אנשים שהמעלית הזו יכולה לאסוף (והיא לא מלאה)
            # הערה: לשיפור נוסף אפשר לבדוק כאן אם יש מקום במעלית (curr_weight + weight_min <= max_w)
            targets_waiting = {p[1] for p in persons if not p[2] and p[1] in reachable_floors}
            
            # ג. קומות "מעבר" (Transfer Floors) - חשוב למקרים שבהם מעלית אחת לא מגיעה ליעד
            # כדי לפשט, אפשר להוסיף את כל הקומות שמשותפות למעליות אחרות
            transfer_floors = set()
            for other_e_id, other_data in self.elevators_static.items():
                if other_e_id != e_id:
                    transfer_floors.update(set(reachable_floors) & set(other_data[1]))

            interesting_floors = targets_inside | targets_waiting | transfer_floors

            for target_floor in interesting_floors:
                if target_floor != e_floor:
                    new_elevs = list(elevators)
                    new_elevs[i] = (e_id, target_floor)
                    action = f"MOVE{{{e_id},{target_floor}}}"
                    successors.append((action, (tuple(new_elevs), persons)))
                    
        # 2. ENTER Actions
        for i, (e_id, e_floor) in enumerate(elevators):
            # Calculate currect weight. count person that is inside an elevator and 
            curr_weight = sum(self.persons_static[p[0]][1] for p in persons if p[2] and p[1] == e_id)
            max_w = self.elevators_static[e_id][2]
            
            for j, (p_id, p_loc, is_in) in enumerate(persons):
                if not is_in and p_loc == e_floor:
                    p_weight = self.persons_static[p_id][1]
                    if curr_weight + p_weight <= max_w: # check person can enter without violating waight constraints
                        new_persons = list(persons)
                        new_persons[j] = (p_id, e_id, True)
                        action = f"ENTER{{{p_id},{e_id}}}"
                        successors.append((action, (elevators, tuple(new_persons))))
                        
        # 3. EXIT Actions
        for i, (e_id, e_floor) in enumerate(elevators):
            for j, (p_id, p_loc, is_in) in enumerate(persons):
                if is_in and p_loc == e_id: # check person is inside elevator and location is e_id
                    new_persons = list(persons)
                    new_persons[j] = (p_id, e_floor, False)
                    action = f"EXIT{{{p_id},{e_id}}}"
                    successors.append((action, (elevators, tuple(new_persons))))
                    
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

    # def h_astar(self, node):
    #     """ This is the heuristic. It gets a node (not a state)
    #     and returns a goal distance estimate"""
        
    #     state = node.state
    #     elevators, persons = state
    #     h = 0
        
    #     for p_id, loc, is_in in persons:
    #         goal_floor = self.persons_static[p_id][2]
            
    #         if is_in:
    #             # האדם כבר במעלית, הוא חייב לצאת כדי לסיים
    #             h += 1
    #         elif loc != goal_floor:
    #             # האדם בקומה הלא נכונה, הוא חייב להיכנס ולצאת
    #             h += 2
    #     return h
    
    def h_astar(self, node):
        state = node.state
        elevators, persons = state
        h = 0
        
        for p_id, loc, is_in in persons:
            p_data = self.persons_static[p_id]
            f_start, weight, f_goal = p_data
            
            # אם הוא כבר ביעד (ומחוץ למעלית), עלות 0
            if not is_in and loc == f_goal:
                continue
                
            if is_in:
                # אדם בתוך מעלית:
                e_id = loc
                reachable_by_current = self.elevators_static[e_id][1]
                if f_goal in reachable_by_current:
                    h += 1 # רק EXIT
                else:
                    h += 3 # חייב EXIT, אז ENTER למעלית אחרת ו-EXIT (סה"כ 3 פעולות)
            else:
                # אדם מחוץ למעלית בקומה loc:
                # האם יש מעלית שיכולה לקחת אותו מ-loc ל-f_goal?
                can_go_direct = any(f_goal in edata[1] and loc in edata[1] 
                                    for edata in self.elevators_static.values())
                if can_go_direct:
                    h += 2 # ENTER + EXIT
                else:
                    h += 4 # ENTER, EXIT, ENTER, EXIT (מינימום להחלפת מעלית)
        return h
        utils.raiseNotDefined()


def create_elevators_problem(game):
    print("<<create_elevators_problem")
    """ Create an elevators problem, based on the description.
    game - tuple of tuples as described in pdf file"""
    return ElevatorsProblem(game)


if __name__ == '__main__':
    ex1_check.main()
