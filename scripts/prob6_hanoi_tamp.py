from re import M
from bmp import *
from bmp.planner.pwuct import PWUCT
from pyperplan.task import Operator, Task
from bmp.planner.bmp import *

if __name__ == "__main__":
    from time import sleep

    dom = DomainHanoi(gui=True)
    prob = ProblemHanoi(dom)
    domain_file = Path(__file__).parent.parent / "bmp/domain/hanoi/pddl/domain.pddl"
    problem_file = Path(__file__).parent.parent / "bmp/domain/hanoi/pddl/problem_ex.pddl"
    
    for i in range(5):
        start = time.perf_counter()
        pwuct = PWUCT(
            domain_file=domain_file.as_posix(),
            problem_file=problem_file.as_posix()
        )
        
        bmp = BMP(
            task=pwuct.task,
            state_disc=pwuct.root.state, 
            mode_init=prob.mode_init, 
            config_init=prob.config_init,
            domain=dom
        )
        finished = False
        while not finished:
            actions = pwuct.get_skeleton()
            result = bmp.plan(actions, dom)
            if result is None:
                reward = bmp.max_fwd_stage / (len(actions))
                pwuct.backpropagate(reward, actions)
            else:
                finished = True
                end = time.perf_counter()
                print(f"iter:{i} elapsed:{end-start}")
                # states, trajs = result
                # for i, traj in enumerate(trajs):
                #     state = states[i]
                #     for config in traj:
                #         dom.assign(state.mode, config)
                #         sleep(0.05)

    input()

