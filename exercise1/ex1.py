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
            pers_list.append((p_id, f_start, False)) # (p_id, current_floor, is_in_elevator) 
            
        # Sort lists in order to get same hash value for all state's permutations
        initial_state = (tuple(sorted(elev_list)), tuple(sorted(pers_list)))
        search.Problem.__init__(self, initial_state)
        
        """ Constructor only needs the initial state.
        Don't forget to set the goal or implement the goal test"""
        search.Problem.__init__(self, initial)

    def successor(self, state):
        """ Generates the successor states returns [(action, achieved_states, ...)]"""
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
        """ This is the heuristic. It gets a node (not a state)
        and returns a goal distance estimate"""
        utils.raiseNotDefined()


def create_elevators_problem(game):
    print("<<create_elevators_problem")
    """ Create an elevators problem, based on the description.
    game - tuple of tuples as described in pdf file"""
    print(game)
    return ElevatorsProblem(game)


if __name__ == '__main__':
    ex1_check.main()
