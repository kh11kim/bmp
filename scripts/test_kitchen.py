from bmp import *
from bmp.domain.kitchen.pddl.pddlstream_prob_kitchen import pddlstream_from_problem, pddlstream_solve, visualize
import time

from bmp.planner.rgh import RGH_rp

if __name__ == "__main__":
    num_block = 5
    dom = DomainKitchen(gui=True, num_box=num_block)
    prob = ProblemKitchen(dom, num_block=num_block, cooked_list=[i+1 for i in range(num_block)])
    domain_file = Path(__file__).parent.parent / "bmp/domain/kitchen/pddl/domain.pddl"
    problem_file = Path(__file__).parent.parent / f"bmp/domain/kitchen/pddl/problem_box{num_block}.pddl"
    domain_file_pddlstream = Path(__file__).parent.parent / "bmp/domain/kitchen/pddl/pddlstream_domain.pddl"
    stream_file_pddlstream = Path(__file__).parent.parent / "bmp/domain/kitchen/pddl/pddlstream_stream.pddl"
    pddlstream_prob = pddlstream_from_problem(
        domain_file_pddlstream.as_posix(), 
        stream_file_pddlstream.as_posix(), 
        prob,
        num_box=num_block
    )
    rgh_orig = RGH(domain_file, problem_file, dom, prob, use_reward=True, mmp_iter=2)
    rgh_no_r = RGH(domain_file, problem_file, dom, prob, use_reward=False, mmp_iter=2)
    rgh_no_goal = RGH(domain_file, problem_file, dom, prob, use_reward=False, mmp_iter=2, no_goal_sampling=True)
    #rgh_random_p = RGH_rp(domain_file, problem_file, dom, prob, use_reward=False)

    
    planner = "rgh_no_goal"
    viz = False
    iteration = 30
    
    result = []
    for i in range(iteration):
        rgh_orig = RGH(domain_file, problem_file, dom, prob, use_reward=True)
        rgh_no_r = RGH(domain_file, problem_file, dom, prob, use_reward=False)
        rgh_no_goal = RGH(domain_file, problem_file, dom, prob, use_reward=True, no_goal_sampling=True)
        
        start = time.perf_counter()
        if planner == "pddlstream":
            with no_output():
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
    
    from output_print import *
    test_name = f"kitchen{num_block}_{planner}_iter{len(result)}"
    save(result, test_name)


    # if viz:
    #     if planner == "pddlstream":
    #         visualize(prob, plan)
    #     elif planner == "rgh":
    #         rgh_orig.visualize()
    #     elif planner == "rgh_no_r":
    #         rgh_no_r.visualize()
    #     elif planner == "rgh_random_p":
    #         rgh_random_p.visualize()
    