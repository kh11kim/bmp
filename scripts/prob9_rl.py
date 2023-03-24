from bmp import *
import time


if __name__ == "__main__":
    num_block = 3
    dom = DomainKitchen(gui=True, num_box=num_block)
    prob = ProblemKitchen(dom, num_block=num_block, cooked_list=[i+1 for i in range(num_block)])
    domain_file = Path(__file__).parent.parent / "bmp/domain/kitchen/pddl/domain.pddl"
    problem_file = Path(__file__).parent.parent / f"bmp/domain/kitchen/pddl/problem_box{num_block}.pddl"
    
    rgh_orig = RGH(domain_file, problem_file, dom, prob, use_reward=True, mmp_iter=2)
    
    viz = True
    iteration = 30
    rgh_orig = RGH(domain_file, problem_file, dom, prob, use_reward=True)
    plan = rgh_orig.plan()
    
    
    # if viz:
    #     if planner == "pddlstream":
    #         visualize(prob, plan)
    #     elif planner == "rgh":
    #         rgh_orig.visualize()
    #     elif planner == "rgh_no_r":
    #         rgh_no_r.visualize()
    #     elif planner == "rgh_random_p":
    #         rgh_random_p.visualize()
    