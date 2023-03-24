from bmp import *
from bmp.planner.pwuct import PWUCT
from pyperplan.task import Operator

def convert_actions(actions:List[Operator]):
    converted = []
    for op in actions:
        if "pick" in op.name:
            name, obj, parent_from, parent_to = op.name.strip("()").split(" ")
            converted += [Pick(obj, parent_from, parent_to)]
        elif "place" in op.name:
            name, obj, parent_from, parent_to = op.name.strip("()").split(" ")
            converted += [Place(obj, parent_from, parent_to)]
    return converted

if __name__ == "__main__":
    from time import sleep

    dom = DomainHanoi(gui=True)
    prob = ProblemHanoi(dom)
    domain_file = Path(__file__).parent.parent / "bmp/domain/hanoi/pddl/domain.pddl"
    problem_file = Path(__file__).parent.parent / "bmp/domain/hanoi/pddl/problem_ex.pddl"
    pwuct = PWUCT(
        domain_file=domain_file.as_posix(),
        problem_file=problem_file.as_posix()
    )
    # actions = [
    #     Pick("box2", "dish1", "robot"),
    #     Place("box2", "robot", "sink1"),
    #     Pick("box2", "sink1", "robot"),
    #     Place("box2", "robot", "oven1"),
    #     Pick("box1", "dish1", "robot"),
    #     Place("box1", "robot", "sink1"),
    #     Pick("box1", "sink1", "robot"),
    #     Place("box1", "robot", "oven1"),
    # ]

    # actions2 = [
    #     Pick("disk1", "disk2", "robot"),
    #     Place("disk1", "robot", "peg3"),
    #     Pick("disk2", "disk3", "robot"),
    #     Place("disk2", "robot", "peg2"),
    #     Pick("disk1", "peg3", "robot"),
    #     Place("disk1", "robot", "disk2"),
    #     Pick("disk3", "peg1", "robot"),
    #     Place("disk3", "robot", "peg3"),
    #     Pick("disk1", "disk2", "robot"),
    #     Place("disk1", "robot", "peg1"),
    #     Pick("disk2", "peg2", "robot"),
    #     Place("disk2", "robot", "disk3"),
    #     Pick("disk1", "peg1", "robot"),
    #     Place("disk1", "robot", "disk2"),
    # ]

    #for i in range(5):
    #make mode family nodes from action sequence
    
    state_init = StateNode(prob.mode_init, prob.config_init, stage=0)
    mftree = ModeFamilyTree(state_init)
    bmp = BMP()
    
    
    start = time.perf_counter()
    
    while True:
        actions = pwuct.get_skeleton()
        actions1 = convert_actions(actions)
        mftree.add_mode_families_by_action_seq(actions2)

        skeleton = mftree.get_skeleton()
        result = bmp.plan(mftree, skeleton, dom)
        if isinstance(result, tuple): break
        max_fwd_stage = result
        reward = max_fwd_stage / (len(skeleton) -1)
        pwuct.backpropagate(reward, actions)

    #end = time.perf_counter()
    #print(f"elapsed: {end-start}")
    
    states, trajs = result
    for i, traj in enumerate(trajs):
        state = states[i]
        for config in traj:
            dom.assign(state.mode, config)
            sleep(0.05)


    input()