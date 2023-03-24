from bmp import *
from bmp.domain.blocktower.pddl.pddlstream_prob_blocktower import pddlstream_from_problem, pddlstream_solve, visualize

import time
from output_print import *
from bmp.planner.rgh import RGH_rp

if __name__ == "__main__":
    
    

    viz = False
    planners = ["rgh_no_goal"]  #"rgh_random_p", "pddlstream", "rgh", "rgh_no_r",
    
    # for planner in planners:
    num_block = 6
    iteration = 30

    dom = DomainBlocktower(gui=True, num_block=num_block)
    prob = ProblemBlocktower(dom)
    domain_file = Path(__file__).parent.parent / "bmp/domain/blocktower/pddl/domain.pddl"
    problem_file = Path(__file__).parent.parent / f"bmp/domain/blocktower/pddl/problem_{num_block}.pddl"
    domain_file_pddlstream = Path(__file__).parent.parent / "bmp/domain/blocktower/pddl/pddlstream_domain2.pddl"
    stream_file_pddlstream = Path(__file__).parent.parent / "bmp/domain/blocktower/pddl/pddlstream_stream2.pddl"
    pddlstream_prob = pddlstream_from_problem(
        domain_file_pddlstream.as_posix(), 
        stream_file_pddlstream.as_posix(), 
        prob,
        num_block=num_block
    )
    
    for planner in planners:
        result = []
        print(f"planner start: {planner}")
        for i in range(iteration):
            
            rgh_orig = RGH(domain_file, problem_file, dom, prob, use_reward=True, mmp_iter=2)
            rgh_no_r = RGH(domain_file, problem_file, dom, prob, use_reward=False, mmp_iter=2)
            rgh_no_goal = RGH(domain_file, problem_file, dom, prob, use_reward=True, no_goal_sampling=True)
            
            start = time.perf_counter()
            if planner == "pddlstream":
                plan = pddlstream_solve(pddlstream_prob)
            elif planner == "rgh":
                plan = rgh_orig.plan()
            elif planner == "rgh_no_r":
                plan = rgh_no_r.plan()
            elif planner == "rgh_no_goal":
                plan = rgh_no_goal.plan(timeout=100)
            end = time.perf_counter()
            elapsed = end-start
            print(f"elapsed:{elapsed}")
            result += [elapsed]
    
        test_name = f"blocktower{num_block}_{planner}_iter{len(result)}"
        save(result, test_name)
        print(f"planner end: {planner}")
    print("end")
    