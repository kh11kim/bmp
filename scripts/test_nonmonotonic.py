from bmp import *
from bmp.domain.nonmonotonic.pddl.pddlstream_prob_nonmonotonic import pddlstream_from_problem, pddlstream_solve, visualize
import time

from bmp.planner.rgh import RGH_rp

if __name__ == "__main__":
    num_box = 3
    dom = DomainNonmonotonic(gui=True, num_box=num_box)
    prob = ProblemNonmonotonic(dom)
    domain_file = Path(__file__).parent.parent / "bmp/domain/nonmonotonic/pddl/domain.pddl"
    problem_file = Path(__file__).parent.parent / f"bmp/domain/nonmonotonic/pddl/problem_{num_box}.pddl"
    domain_file_pddlstream = Path(__file__).parent.parent / "bmp/domain/nonmonotonic/pddl/pddlstream_domain.pddl"
    stream_file_pddlstream = Path(__file__).parent.parent / "bmp/domain/nonmonotonic/pddl/pddlstream_stream.pddl"
    pddlstream_prob = pddlstream_from_problem(
        domain_file_pddlstream.as_posix(), 
        stream_file_pddlstream.as_posix(), 
        prob,
        num_box = num_box
        )
    
    from output_print import *

    viz = False
    planners = ["pddlstream"] #, "rgh", "rgh_no_r", "rgh_no_goal", 
    #planner = "pddlstream"
    #planner = "rgh"
    # for planner in planners:
    iteration = 30
    timeout = 100
    for planner in planners:
        result = []
        for i in range(iteration):
            rgh_orig = RGH(domain_file, problem_file, dom, prob, use_reward=True, mmp_iter=2)
            rgh_no_r = RGH(domain_file, problem_file, dom, prob, use_reward=False, mmp_iter=2)
            rgh_no_goal = RGH(domain_file, problem_file, dom, prob, use_reward=True, no_goal_sampling=True)
            
            start = time.perf_counter()
            if planner == "pddlstream":
                plan = pddlstream_solve(pddlstream_prob, timeout=timeout)
            elif planner == "rgh":
                plan = rgh_orig.plan(timeout=timeout)
            elif planner == "rgh_no_r":
                plan = rgh_no_r.plan(timeout=timeout)
            elif planner == "rgh_no_goal":
                plan = rgh_no_goal.plan(timeout=timeout)
            end = time.perf_counter()
            elapsed = end-start
            print(f"elapsed:{elapsed}")
            result += [elapsed]
        
        
            test_name = f"nonmonotonic{num_box}_{planner}_iter{len(result)}"
            save(result, test_name)
    