from bmp import *
import time

task = "kitchen"

if task == "kitchen":
    num_block = 2
    dom = DomainKitchen(gui=True, num_box=num_block)
    prob = ProblemKitchen(dom, num_block=num_block, cooked_list=[i+1 for i in range(num_block)])
    domain_file = Path(__file__).parent.parent / "bmp/domain/kitchen/pddl/domain.pddl"
    problem_file = Path(__file__).parent.parent / f"bmp/domain/kitchen/pddl/problem_box{num_block}.pddl"


mode_init = deepcopy(prob.mode_init)
mode = prob.mode_init
config = prob.config_init
for i in range(100):
    p1 = dom.sample_placement("box1", "sink1")
    p2 = dom.sample_placement("box2", "sink1")
    mode.set_attachment(p1)
    mode.set_attachment(p2)
    if dom.is_collision(mode, config):
        break

dom.assign(mode_init)

# 1
mode_1 = deepcopy(mode_init)
mode_1.set_attachment(p1)
dom.assign(mode_1)


#2
mode_2 = deepcopy(mode_init)
mode_2.set_attachment(p2)
dom.assign(mode_2)

#3
mode_3 = deepcopy(mode_init)
mode_3.set_attachment(p1)
mode_3.set_attachment(p2)
dom.assign(mode_3)

input()