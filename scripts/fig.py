from bmp import *
import time
from bmp.domain.regrasping.regrasping import DomainRegrasping, ProblemRegrasping

from bmp.planner.rgh import RGH_rp

task = "sorting"

if task == "kitchen":
    num_block = 2
    dom = DomainKitchen(gui=True, num_box=num_block)
    prob = ProblemKitchen(dom, num_block=num_block, cooked_list=[i+1 for i in range(num_block)])
    domain_file = Path(__file__).parent.parent / "bmp/domain/kitchen/pddl/domain.pddl"
    problem_file = Path(__file__).parent.parent / f"bmp/domain/kitchen/pddl/problem_box{num_block}.pddl"
elif task == "nonmonotonic":
    num_box = 2
    dom = DomainNonmonotonic(gui=True, num_box=num_box)
    prob = ProblemNonmonotonic(dom)
    domain_file = Path(__file__).parent.parent / "bmp/domain/nonmonotonic/pddl/domain.pddl"
    problem_file = Path(__file__).parent.parent / f"bmp/domain/nonmonotonic/pddl/problem_{num_box}.pddl"
elif task == "blocktower":
    num_block = 4
    dom = DomainBlocktower(gui=True, num_block=num_block)
    prob = ProblemBlocktower(dom)
    domain_file = Path(__file__).parent.parent / "bmp/domain/blocktower/pddl/domain.pddl"
    problem_file = Path(__file__).parent.parent / f"bmp/domain/blocktower/pddl/problem_{num_block}.pddl"
elif task == "hanoi":
    dom = DomainHanoi(gui=True)
    prob = ProblemHanoi(dom)
    domain_file = Path(__file__).parent.parent / "bmp/domain/hanoi/pddl/domain.pddl"
    problem_file = Path(__file__).parent.parent / f"bmp/domain/hanoi/pddl/problem.pddl"
elif task == "sorting":
    dom = DomainSorting(gui=True)
    prob = ProblemSorting(dom)
    domain_file = Path(__file__).parent.parent / "bmp/domain/sorting/pddl/domain.pddl"
    problem_file = Path(__file__).parent.parent / f"bmp/domain/sorting/pddl/problem.pddl"

elif task == "regrasping":
    num_dice=1
    dom = DomainRegrasping(gui=True, num_dice=num_dice)
    prob = ProblemRegrasping(dom)
    domain_file = Path(__file__).parent.parent / "bmp/domain/regrasping/pddl/domain.pddl"
    problem_file = Path(__file__).parent.parent / f"bmp/domain/regrasping/pddl/problem_{num_dice}.pddl"


rgh_orig = RGH(domain_file, problem_file, dom, prob, eps_branch=0.3, planner="wastar", use_reward=True)
an = rgh_orig.plan()
rgh_orig.visualize()