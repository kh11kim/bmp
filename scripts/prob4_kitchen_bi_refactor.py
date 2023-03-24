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

    dom = DomainKitchen(gui=True, num_box=3)
    prob = ProblemKitchen(dom, num_block=3, cooked_list=[1, 2])
    domain_file = Path(__file__).parent.parent / "bmp/domain/kitchen/pddl/domain.pddl"
    problem_file = Path(__file__).parent.parent / "bmp/domain/kitchen/pddl/problem_ex.pddl"
    pwuct = PWUCT(
        domain_file=domain_file.as_posix(),
        problem_file=problem_file.as_posix()
    )
    


    #for i in range(5):
    #make mode family nodes from action sequence
    state_init = StateNode(prob.mode_init, prob.config_init, stage=0)
    mftree = ModeFamilyTree(state_init)
    bmp = BMP()
    
    
    #start = time.perf_counter()
    
    while True:
        actions = pwuct.get_skeleton()
        actions1 = convert_actions(actions)
        mftree.add_mode_families_by_action_seq(actions1)
        #skeleton = mftree.get_skeleton_by_action(actions1)
        skeleton = mftree.get_skeleton()
        result = bmp.plan(mftree, skeleton, dom)
        if isinstance(result, tuple): break
        max_fwd_stage = result
        reward = max_fwd_stage / (len(skeleton) -1)
        pwuct.backpropagate(reward, actions)
    end = time.perf_counter()
    #print(f"elapsed: {end-start}")
    
    states, trajs = result
    for i, traj in enumerate(trajs):
        state = states[i]
        for config in traj:
            dom.assign(state.mode, config)
            sleep(0.05)


    input()
    # actions2 = [
    #     Pick("box2", "dish1", "robot"),
    #     Place("box2", "robot", "sink1"),
    #     Pick("box2", "sink1", "robot"),
    #     Place("box2", "robot", "oven1"),
    #     Pick("box1", "dish1", "robot"),
    #     Place("box1", "robot", "sink1"),
    #     Pick("box2", "oven1", "robot"),
    #     Place("box2", "robot", "dish1"),
    #     Pick("box1", "sink1", "robot"),
    #     Place("box1", "robot", "oven1"),
    #     Pick("box0", "dish1", "robot"),
    #     Place("box0", "robot", "sink1"),
    #     Pick("box1", "oven1", "robot"),
    #     Place("box1", "robot", "dish1"),
    #     Pick("box0", "sink1", "robot"),
    #     Place("box0", "robot", "oven1"),
    # ]