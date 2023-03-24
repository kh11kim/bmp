from bmp import *
from bmp.domain.unpacking.pddl.pddlstream_prob_unpacking import pddlstream_from_problem, pddlstream_solve, visualize
import time

from bmp.planner.rgh import RGH_rp

if __name__ == "__main__":
    dom = DomainUnpacking(gui=True)
    prob = ProblemUnpacking(dom)
    domain_file = Path(__file__).parent.parent / "bmp/domain/unpacking/pddl/domain.pddl"
    problem_file = Path(__file__).parent.parent / "bmp/domain/unpacking/pddl/problem.pddl"
    domain_file_pddlstream = Path(__file__).parent.parent / "bmp/domain/unpacking/pddl/pddlstream_domain.pddl"
    stream_file_pddlstream = Path(__file__).parent.parent / "bmp/domain/unpacking/pddl/pddlstream_stream.pddl"
    pddlstream_prob = pddlstream_from_problem(
        domain_file_pddlstream.as_posix(), 
        stream_file_pddlstream.as_posix(), 
        prob)
    rgh_orig = RGH(domain_file, problem_file, dom, prob, use_reward=True, mmp_iter=2, eps=0.5)
    rgh_no_r = RGH(domain_file, problem_file, dom, prob, use_reward=False, mmp_iter=2)
    rgh_random_p = RGH_rp(domain_file, problem_file, dom, prob, use_reward=False)

    # setting
    planner = "rgh_no_r"
    viz = True


    start = time.perf_counter()
    if planner == "pddlstream":
        plan = pddlstream_solve(pddlstream_prob)
    elif planner == "rgh":
        plan = rgh_orig.plan()
    elif planner == "rgh_no_r":
        plan = rgh_no_r.plan()
    elif planner == "rgh_random_p":
        plan = rgh_random_p.plan()
    end = time.perf_counter()
    print(f"elapsed:{end-start}")
    
    if viz:
        if planner == "pddlstream":
            visualize(prob, plan)
        elif planner == "rgh":
            rgh_orig.visualize()
        elif planner == "rgh_no_r":
            rgh_no_r.visualize()
        elif planner == "rgh_random_p":
            rgh_random_p.visualize()

    # rgh_no_r.plan()
    
    
    #rgh_no_r.visualize()
    