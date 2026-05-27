import time
import ex1
import search as search
import utils as utils

def run_problem(func, targs=(), kwargs=None):
    if kwargs is None:
        kwargs = {}
    result = (-3, "default")
    try:
        result = func(*targs, **kwargs)
    except Exception as e:
        result = (-3, e)
    return result

def solve_problems(problem):
    try:
        p = ex1.create_elevators_problem(problem)
    except Exception as e:
        print("Error creating problem: ", e)
        return None

    result = run_problem((lambda p: search.astar_search(p, p.h_astar)), targs=[p])

    if result and isinstance(result[0], search.Node):
        solve = result[0].path()[::-1]
        solution = [pi.action for pi in solve][1:]
        # Print the solution path
        # print(solution) 
        return len(solution)
    else:
        print("no solution")
        return None


# =====================================================================
# 1. ORIGINAL & EASY & MEDIUM PROBLEMS
# =====================================================================

init_state_p1 = {
    "height": 6,
    "Elevators": {
        0: (0, (0, 1, 2, 3), 8),
        1: (4, (2, 4, 5, 6), 10)
    },
    "Persons": {
        10: (0, 3, 3),
        11: (2, 4, 6),
        12: (4, 5, 0)
    }
}

init_state_e1 = {
    "height": 5,
    "Elevators": {
        0: (0, (0, 1, 2, 3, 4, 5), 15),
        1: (5, (0, 1, 2, 3, 4, 5), 15)
    },
    "Persons": {
        10: (0, 3, 5), 11: (5, 3, 0), 12: (3, 3, 1)
    }
}

init_state_e2 = {
    "height": 6,
    "Elevators": {
        0: (0, (0, 1, 2, 3), 10),
        1: (6, (3, 4, 5, 6), 10)
    },
    "Persons": {
        10: (1, 3, 3), 11: (5, 3, 4), 12: (0, 3, 2)
    }
}

init_state_e3 = {
    "height": 6,
    "Elevators": {
        0: (0, (0, 1, 2, 3, 4), 10),
        1: (6, (2, 4, 5, 6), 10)
    },
    "Persons": {
        10: (0, 3, 5), 11: (6, 3, 1), 12: (3, 4, 6)
    }
}

init_state_e4 = {
    "height": 5,
    "Elevators": {
        0: (0, (0, 1, 2, 3, 4, 5), 7),
        1: (5, (0, 1, 2, 3, 4, 5), 7)
    },
    "Persons": {
        10: (0, 5, 5), 11: (0, 5, 3), 12: (5, 5, 0)
    }
}

init_state_e5 = {
    "height": 7,
    "Elevators": {
        0: (0, (0, 1, 2, 3, 4), 12),
        1: (7, (3, 4, 5, 6, 7), 12)
    },
    "Persons": {
        10: (1, 4, 3), 11: (6, 4, 7), 12: (0, 4, 7)
    }
}

init_state_m1 = {
    "height": 6,
    "Elevators": {
        0: (0, (0, 1, 2, 3,), 12),
        1: (6, (3, 4, 5, 6,), 12)
    },
    "Persons": {
        10: (0, 4, 5), 11: (5, 4, 0), 12: (2, 4, 6), 13: (5, 4, 1)
    }
}

init_state_m2 = {
    "height": 8,
    "Elevators": {
        0: (0, (0, 1, 2, 3, 4), 10),
        1: (4, (2, 4, 6, 8), 10)
    },
    "Persons": {
        10: (0, 3, 8), 11: (8, 3, 0), 12: (2, 3, 6), 13: (6, 3, 1)
    }
}

init_state_m3 = {
    "height": 8,
    "Elevators": {
        0: (0, (0, 1, 2, 3, 4), 10),
        1: (4, (2, 4, 6, 8), 10),
        2: (4, (7,2), 10)
    },
    "Persons": {
        10: (0, 3, 8), 11: (8, 3, 0), 12: (2, 3, 7), 13: (6, 3, 1)
    }
}

init_state_m4 = {
    "height": 6,
    "Elevators": {
        0: (0, (0, 1, 2, 3, 4, 5, 6), 8),
        1: (6, (0, 1, 2, 3, 4, 5, 6), 8)
    },
    "Persons": {
        10: (0, 5, 6), 11: (0, 5, 4), 12: (6, 5, 0), 13: (6, 5, 2), 14: (3, 5, 6)
    }
}

init_state_m5 = {
    "height": 8,
    "Elevators": {
        0: (0, (0, 1, 2, 3, 4), 10),
        1: (4, (4, 5, 6, 7, 8), 10)
    },
    "Persons": {
        10: (0, 6, 8), 11: (0, 4, 5), 12: (8, 6, 0), 13: (8, 5, 3)
    }
}

# =====================================================================
# 2. TESTPROBS (From problems.txt)
# =====================================================================

init_state_test_p1 = {
    "height": 8,
    "Elevators": { 0: (0, (0, 1, 2, 3, 4), 10), 1: (4, (2, 4, 6, 8), 10) },
    "Persons": { 10: (0, 3, 8), 11: (8, 3, 0), 12: (2, 3, 6), 13: (6, 3, 1) }
}

init_state_test_p2 = {
    "height": 8,
    "Elevators": { 0: (0, (0, 1, 2, 3, 4), 10), 1: (4, (2, 4, 6, 8), 10), 2: (4, (7, 2), 10) },
    "Persons": { 10: (0, 3, 8), 11: (8, 3, 0), 12: (2, 3, 7), 13: (6, 3, 1) }
}

init_state_test_p3 = {
    "height": 6,
    "Elevators": { 0: (0, (0, 1, 2, 3, 4, 5, 6), 8), 1: (6, (0, 1, 2, 3, 4, 5, 6), 8) },
    "Persons": { 10: (0, 5, 6), 11: (0, 5, 4), 12: (6, 5, 0), 13: (6, 5, 2), 14: (3, 5, 6) }
}

init_state_test_p4 = {
    "height": 8,
    "Elevators": { 0: (0, (0, 1, 2, 3, 4), 10), 1: (4, (4, 5, 6, 7, 8), 10) },
    "Persons": { 10: (0, 6, 8), 11: (0, 4, 5), 12: (8, 6, 0), 13: (8, 5, 3) }
}

init_state_test_p5 = {
    "height": 8,
    "Elevators": { 0: (0, (0, 1, 2, 3, 4), 10), 1: (4, (3, 4, 5), 10), 2: (5, (5, 6, 7, 8), 10) },
    "Persons": { 10: (0, 3, 8), 11: (8, 3, 0), 12: (4, 3, 7), 13: (2, 3, 6) }
}

init_state_test_p6 = {
    "height": 8,
    "Elevators": { 0: (0, (0, 1, 2, 3, 4), 10), 1: (4, (2, 4, 6, 8), 10), 2: (4, (7, 2), 10) },
    "Persons": { 10: (0, 3, 8), 11: (8, 3, 0), 12: (2, 3, 7), 13: (6, 3, 1), 14: (4, 3, 2) }
}

init_state_test_p7 = {
    "height": 14,
    "Elevators": {
        0: (0, (0, 2), 100), 1: (2, (4,), 100), 2: (4, (6,), 100),
        3: (6, (8,), 100), 4: (8, (10,), 100), 5: (10, (12,), 100), 6: (12, (14,), 100)
    },
    "Persons": { 10: (0, 8, 14), 11: (0, 5, 10), 12: (0, 5, 6) }
}

init_state_test_p8 = {
    "height": 8,
    "Elevators": { 0: (0, (0, 1, 2, 3), 9), 1: (4, (3, 4, 5, 6), 9), 2: (8, (6, 7, 8), 9) },
    "Persons": { 33: (0, 3, 8), 34: (8, 3, 0), 35: (2, 2, 6), 36: (6, 2, 1) }
}

init_state_test_p9 = {
    "height": 14,
    "Elevators": {
        0: (0, (0, 3), 100), 1: (3, (6,), 100), 2: (6, (9,), 100),
        3: (9, (12,), 100), 4: (9, (4,), 100), 5: (4, (1,), 100)
    },
    "Persons": { 20: (0, 10, 12), 21: (0, 10, 1) }
}

init_state_test_p10 = {
    "height": 16,
    "Elevators": { 0: (0, (0, 4), 100), 1: (4, (8,), 100), 2: (8, (8, 11, 14), 100), 3: (14, (16,), 100) },
    "Persons": { 30: (0, 10, 16), 31: (8, 10, 11), 32: (8, 10, 14) }
}

# =====================================================================
# 3. BENCH PROBLEMS (From problems.txt)
# =====================================================================

init_state_competition_1 = {
    "height": 6,
    "Elevators": { 0: (0, (0, 1, 2, 3,), 12), 1: (6, (3, 4, 5, 6,), 12) },
    "Persons": { 10: (0, 4, 5), 11: (5, 4, 0), 12: (2, 4, 6), 13: (5, 4, 1) }
}

init_state_competition_2 = {
    "height": 10,
    "Elevators": {
        0: (0, (0, 1, 2, 3, 4, 5), 300),
        1: (5, (2, 3, 4, 5, 6, 7, 8), 300),
        2: (10, (5, 6, 7, 8, 9, 10), 300)
    },
    "Persons": { 40: (0, 70, 10), 41: (2, 70, 8), 42: (10, 70, 0) }
}

init_state_competition_3 = {
    "height": 8,
    "Elevators": {
        0: (0, (0, 1, 2, 3, 4, 5, 6, 7, 8), 200),
        1: (8, (0, 2, 4, 6, 8), 300),
        2: (4, (0, 1, 2, 3, 4), 150)
    },
    "Persons": { 1: (1, 70, 7), 2: (7, 90, 0), 3: (0, 110, 8), 4: (4, 80, 2), 5: (2, 65, 6) }
}

init_state_competition_4 = {
    "height": 7,
    "Elevators": { 0: (0, (0, 1, 2, 3, 4), 12), 1: (7, (4, 5, 6, 7), 12) },
    "Persons": { 10: (0, 4, 6), 11: (6, 4, 1), 12: (2, 4, 7), 13: (5, 4, 0) }
}

init_state_competition_5 = {
    "height": 8,
    "Elevators": { 0: (1, (0, 1, 2, 3, 4), 10), 1: (5, (3, 4, 5, 6, 7, 8), 10) },
    "Persons": { 10: (0, 3, 8), 11: (8, 3, 0), 12: (1, 3, 6), 13: (6, 3, 1) }
}

init_state_competition_6 = {
    "height": 10,
    "Elevators": {
        0: (0, (0, 1, 2, 3, 4), 12),
        1: (4, (4, 5, 6, 7), 12),
        2: (10, (6, 7, 8, 9, 10), 12)
    },
    "Persons": { 40: (0, 3, 10), 41: (10, 3, 0), 42: (2, 3, 8), 43: (7, 3, 1) }
}

init_state_competition_7 = {
    "height": 9,
    "Elevators": { 0: (2, (0, 1, 2, 3, 4, 5), 10), 1: (6, (4, 5, 6, 7, 8, 9), 10) },
    "Persons": { 10: (0, 3, 7), 11: (7, 3, 0), 12: (2, 3, 9), 13: (9, 3, 2) }
}

init_state_competition_8 = {
    "height": 11,
    "Elevators": {
        0: (0, (0, 1, 2, 3, 4, 5), 12),
        1: (5, (4, 5, 6, 7, 8), 12),
        2: (11, (7, 8, 9, 10, 11), 12)
    },
    "Persons": { 40: (0, 3, 11), 41: (11, 3, 0), 42: (3, 3, 6), 43: (8, 3, 1) }
}

init_state_competition_9 = {
    "height": 7,
    "Elevators": { 0: (0, (0, 1, 2, 3, 4), 8), 1: (4, (4, 5, 6, 7), 8) },
    "Persons": {
        10: (0, 4, 7), 11: (7, 4, 0), 12: (1, 3, 5), 
        13: (5, 3, 1), 14: (4, 3, 6), 15: (6, 3, 3)
    }
}

init_state_competition_10 = {
    "height": 9,
    "Elevators": {
        0: (0, (0, 1, 2, 3, 4), 14),
        1: (4, (3, 4, 5, 6, 7), 14),
        2: (9, (7, 8, 9), 14)
    },
    "Persons": { 40: (0, 4, 9), 41: (9, 4, 0), 42: (2, 2, 7), 43: (7, 2, 1) }
}


def main():
    start = time.time()
    
    # List format: (Name, Problem Dictionary, Expected Optimal Length)
    problems = [
        # Original & Easies
        ("p1  (original)", init_state_p1, 13),
        ("e1  (easy)",     init_state_e1, 10),
        ("e2  (easy)",     init_state_e2, 11),
        ("e3  (easy)",     init_state_e3, 18),
        ("e4  (easy)",     init_state_e4, 9),
        ("e5  (easy)",     init_state_e5, 13),
        
        # Mediums
        ("m1  (medium)",   init_state_m1, 24),
        ("m2  (medium)",   init_state_m2, 21),
        ("m3  (medium)",   init_state_m3, 22),
        ("m4  (medium)",   init_state_m4, 16),
        ("m5  (medium)",   init_state_m5, 25),
        
        # TESTPROBS
        ("test_p1",  init_state_test_p1,  21),
        ("test_p2",  init_state_test_p2,  22),
        ("test_p3",  init_state_test_p3,  16),
        ("test_p4",  init_state_test_p4,  25),
        ("test_p5",  init_state_test_p5,  31),
        ("test_p6",  init_state_test_p6,  24),
        ("test_p7",  init_state_test_p7,  37),
        ("test_p8",  init_state_test_p8,  29),
        ("test_p9",  init_state_test_p9,  24),
        ("test_p10", init_state_test_p10, 17),
        
        # BENCH
        ("competition_1",  init_state_competition_1,  24),
        ("competition_2",  init_state_competition_2,  16),
        ("competition_3",  init_state_competition_3,  17),
        ("competition_4",  init_state_competition_4,  25),
        ("competition_5",  init_state_competition_5,  24),
        ("competition_6",  init_state_competition_6,  31),
        ("competition_7",  init_state_competition_7,  24),
        ("competition_8",  init_state_competition_8,  29),
        ("competition_9",  init_state_competition_9,  33),
        ("competition_10", init_state_competition_10, 28)
    ]
    
    print(f"{'PROBLEM':<18} | {'TIME':<8} | {'STEPS':<6} | {'OPTIMAL':<8}")
    print("-" * 50)
    
    for name, p, expected in problems:
        t0 = time.time()
        
        # Run problem and get actual steps count
        actual_steps = solve_problems(p)
        time_taken = time.time() - t0
        
        # Check against expected
        if actual_steps == expected:
            status = "✅ YES"
        elif actual_steps is None:
            status = "❌ FAIL"
        else:
            status = f"❌ NO (Exp: {expected})"
            
        print(f"{name:<18} | {time_taken:5.2f}s  | {actual_steps if actual_steps else '-':<6} | {status}")
        
    end = time.time()
    print("-" * 50)
    print(f"Total submission took: {end - start:.2f} seconds.")

if __name__ == '__main__':
    main()