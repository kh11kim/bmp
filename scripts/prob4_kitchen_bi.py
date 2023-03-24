from pybullet_suite import *
from copy import deepcopy
from typing import Optional, List, Dict, Set
from dataclasses import dataclass, field
import numpy as np
from bmp import *
from symk import run_symk, remove_plan_folder
from pathlib import Path
from time import sleep
import heapq

@dataclass
class State:
    mode: Mode
    config: Config
    parent: Optional["State"] = field(default_factory=lambda :None)
    traj_to_parent: Optional[List[Config]] = field(default_factory=lambda :None)
    traj_switch: Optional[List[Config]] = field(default_factory=lambda :None)
    traj: Optional[List[Config]] = field(default_factory=lambda :None)

@dataclass
class ModeFamily:
    stage: int
    mode_family_str: str
    guides: Optional[List[Attachment]] = field(default_factory=lambda :[])
    is_root: Optional[bool] = field(default_factory=lambda :False)
    states: List[State] = field(default_factory=lambda :[])
    parent: Optional["ModeFamily"] = field(default_factory=lambda :None)
    action_from_parent: Optional[Attach] = field(default_factory=lambda :None)
    childs: Dict[str,"ModeFamily"] = field(default_factory=lambda :{})
    desired_config: Optional[Config] = field(default_factory=lambda :None)

    def __repr__(self):
        return f"{self.mode_family_str}"

    @property
    def numbered_var_dict(self):
        numbered_var_list = self.mode_family_str.split("_")[1:]
        result = {}
        for numbered_var in numbered_var_list:
            obj, parent, num = numbered_var.split("-")
            result[obj] = numbered_var
        return result

    @property
    def var_dict(self):
        numbered_var_list = self.mode_family_str.split("_")[1:]
        result = {}
        for numbered_var in numbered_var_list:
            obj, parent, num = numbered_var.split("-")
            var = "-".join([obj, parent])
            result[obj] = var
        return result
    
    def get_var_to_sample(self):
        return self.numbered_var_dict[self.action_from_parent.obj_name]

class ModeFamilyTree:
    def __init__(self):
        self.V: Dict[int, ModeFamily] = {0:[]}
        self.variables = []

    @property
    def root(self):
        return self.V[0][0]

    def get_key(self, var):
        # var (str): obj-parent
        num = 0
        while True:
            key = f"{var}-{num}"
            if key not in self.variables:
                break
            num += 1
        return key
    
    def run_task_planner(self, k: int, domain_file:str, problem_file:str, plan_per_k=100):
        remove_plan_folder()
        run_symk(
            domain_file, problem_file,
            num_plan=k*plan_per_k
        )
    
    def add_task_plan(self, k: int, state_init:Optional[State]=None):
        plan_path = list(Path("./found_plans").glob(f"sas_plan.{k}"))[0]
        actions = parse_action(plan_path)
        num_new_nodes = self.parse_mode_families_by_action(actions, state_init)
        
        # if k == 1:
        #     assert state_init is not None
        #     self.root.is_root = True
        #     self.root.states.append(state_init)
        if num_new_nodes != 0:
            return True
        return None

    def add_mode_family(
        self,
        new_var_dict:Dict[str, str], #state: Dict[obj, var]
        parent:Optional[ModeFamily]=None, 
        is_root=False
    ):
        def get_new_mode_family_str(stage:int, parent: ModeFamily, new_var_dict:Dict, is_root=False):
            keys = [str(stage)]
            if not is_root:
                numbered_var_list = parent.numbered_var_dict
                prev_var_dict = parent.var_dict
            else:
                prev_var_dict = {}
            #same_vars = set(prev_var_dict.values()).intersection(new_var_dict.values())
            new_vars = set(new_var_dict.values()).difference(prev_var_dict.values())
            for obj in new_var_dict.keys():
                if new_var_dict[obj] not in new_vars:
                    keys.append(numbered_var_list[obj])
                else:
                    var = new_var_dict[obj]
                    key = self.get_key(var)
                    self.variables.append(key)
                    keys.append(key)    
            return "_".join(keys)

        var_str = self.make_var_str(new_var_dict)
        if is_root:
            stage, parent = 0, None
        else:
            stage = parent.stage + 1 
        
        if stage not in self.V:
            self.V[stage] = []
        
        mode_family_str = get_new_mode_family_str(stage, parent, new_var_dict, is_root=is_root)
        new_node = ModeFamily(stage, mode_family_str, parent=parent)
        self.V[stage].append(new_node)
        if not is_root:
            parent.childs[var_str] = new_node
        return new_node
    
    def get_skeleton(self):
        root:ModeFamily = self.V[0][0]
        skeleton =[root]
        node = root
        while True:
            keys = list(node.childs.keys())
            key = np.random.choice(keys)
            if key == "end":
                break
            node = node.childs[key]
            skeleton.append(node)
        return skeleton


    def parse_mode_families_by_action(self, actions: List[Attach], state_init: Optional[State]=None):
        """Return a number of new nodes

        :param mode_init: _description_
        :type mode_init: Mode
        :param actions: _description_
        :type actions: List[Attach]
        :return: _description_
        :rtype: _type_
        """
        # init
        if len(self.V[0]) == 0:    
            assert state_init is not None
            var_dict = {}
            for obj, att in state_init.mode.attachments.items():
                parent = att.parent_name
                var_dict[obj]= f"{obj}-{parent}"
            root: ModeFamily = self.add_mode_family(var_dict, is_root=True)
            assert state_init is not None
            root.is_root = True
            root.states.append(state_init)
        else: 
            root = self.root
            var_dict = root.var_dict

        node = root
        nodes = [root]
        new_nodes = 0
        for action in actions:
            obj, parent = action.obj_name, action.parent_to
            var_dict[obj] = f"{obj}-{parent}"
            var_str = self.make_var_str(var_dict)
            if var_str not in node.childs:
                node = self.add_mode_family(var_dict, node)
                node.action_from_parent = action
                new_nodes += 1
            else:
                node = node.childs[var_str]
            nodes.append(node)
        node.desired_config = root.states[0].config
        node.childs["end"] = None
        return new_nodes

    def make_var_str(self, var_dict:Dict):
        variables = []
        for obj, var in var_dict.items():
            variables.append(var)
        return "_".join(variables)

def parse_action(path:Path):
    with open(path, 'r') as f:
        lines = f.readlines()
    action_list = []
    for line in lines:
        if line[0] != ";":
            action_name, obj_name, region_name = line.strip("()\n").split(" ")
            if action_name == "pick":
                a = Pick(obj_name, parent_from=region_name, parent_to="robot")
                action_list.append(a)
            elif action_name == "place":
                a = Place(obj_name, parent_from="robot", parent_to=region_name)
                action_list.append(a)
    return action_list

def make_random_feasible_goal_state(
    state_ref:State, changed_atts:Set[str],
    domain:TAMPDomain, config_ref:Optional[Config]=None
) -> State:
    mode_new = deepcopy(state_ref.mode)
    config_new = deepcopy(state_ref.config)
    for i in range(100):
        for numbered_var in changed_atts:
            obj, parent, _ = numbered_var.split("-")
            if "robot" in parent:
                att = domain.sample_grasp(obj, parent)
            else:
                att = domain.sample_placement(obj, parent)
            mode_new.set_attachment(att)
        if config_ref is None:
            config_new.q["robot"] = domain.robots["robot"].get_random_joint_angles()
        else:
            config_new = config_ref
        domain.assign(mode_new, config_new)
        if not domain.is_collision(mode_new, config_new):
            return State(mode_new, config_new)
    return None

def main():
    domain = DomainKitchen(gui=True, num_box=5)
    problem = ProblemKitchen(domain, num_block=5, cooked_list=[0,1,2])

    mode_init = problem.mode_init
    config_init = problem.config_init
    state_init = State(mode_init, config_init)

    mf_tree = ModeFamilyTree()
    domain_file, problem_file = problem.make_temp_pddl_files()

    start = time.perf_counter()
    mf_tree.run_task_planner(1, domain_file, problem_file, plan_per_k=10000)
    #mf_tree.add_task_plan(1, state_init)
    
    skeleton = [
        Pick("box1", "dish1", "robot"),
        Place("box1", "robot", "sink1"),
        Pick("box1", "sink1", "robot"),
        Place("box1", "robot", "oven1"),
    ]
    num_new_nodes = mf_tree.parse_mode_families_by_action(skeleton, state_init)



    running = True
    for k in range(2, 100000):
        if not running: break
        #if mf_tree.add_task_plan(k) is None: continue

        skeleton = mf_tree.get_skeleton() #TODO: MCTS


        stage_start = 0
        milestones = divide_stages(skeleton, max_len=3)
        for stage_end in milestones:    
            state_new = grow_to_milestone(
                skeleton, 
                stage_start, stage_end, 
                domain, max_iter=10
            )
            if state_new is None: break
            stage_start = stage_end
        if state_new is not None:
            finish = time.perf_counter()
            print(f"elapsed: {finish-start} sec")
            running = False
            print("finish")

    states = backtrack(state_new)
    for state in states:
        mp = BiRRT2()
        mp.init(["robot"], domain, state.mode)
        for config in mp.smoothing(state.traj):
            domain.assign(state.mode, config)
            sleep(0.1)
        
    input()

def divide_stages(
    skeleton: List[ModeFamily],
    max_len: int = 3
):
    # given skeleton, we can calculate goal configuration
    init_var_set = set(skeleton[0].numbered_var_dict.values())
    goal_var_set = set(skeleton[-1].numbered_var_dict.values())
    known_var_set = init_var_set.union(goal_var_set)
    
    milestones = []
    for i, node in enumerate(skeleton[1:], start=1):
        node_vars = set(node.numbered_var_dict.values())
        if node_vars.difference(known_var_set) == set():
            #milestone
            milestones.append(i)
    
    result = []
    start = 0
    for milestone in milestones:
        if milestone - start >= max_len:
            new_milestones = [*range(start, milestone, max_len), milestone][1:]
            result.extend(new_milestones)
        else: 
            result.append(milestone)
        start = milestone
    return result
    


def grow_to_milestone(
    skeleton:List[ModeFamily], 
    stage_start:int,
    stage_end:int,
    domain:TAMPDomain,
    max_iter=100
):
    # input : skeleton, goal_stage, goal_state
    # output : enlarged fwd_tree(skeleton)

    # given skeleton, we can calculate goal configuration
    mf_init = skeleton[stage_start]
    mf_goal = skeleton[stage_end]
    state_init = np.random.choice(mf_init.states)
    init_var_set = set(mf_init.numbered_var_dict.values())
    goal_var_set = set(mf_goal.numbered_var_dict.values())
    changed_atts = goal_var_set.difference(init_var_set)
    
    state_goal_sampled = make_random_feasible_goal_state(
        state_init, changed_atts, domain, config_ref=skeleton[stage_end].desired_config)
    if state_goal_sampled is None: return None
    
    #make backward skeleton
    state_inits = skeleton[stage_start].states
    skeleton_bwd: List[ModeFamily] = []
    for stage in range(stage_end+1):
        if stage < stage_start:
            skeleton_bwd.append(None)
            continue
        fwd_node = skeleton[stage]
        mf = ModeFamily(stage, fwd_node.mode_family_str)
        skeleton_bwd.append(mf)
    for node in skeleton_bwd[stage_start:stage_end]:
        node.parent = skeleton_bwd[node.stage + 1]
        node.action_from_parent = skeleton[node.stage + 1].action_from_parent.reverse()
    skeleton_bwd[stage_end].is_root = True
    skeleton_bwd[stage_end].states.append(state_goal_sampled)
    att_end = state_goal_sampled.mode.attachments[skeleton[stage_end].action_from_parent.obj_name]
    
    skeleton[stage_end].guides.append(att_end)
    for state_init in state_inits:
        obj = skeleton_bwd[stage_start].action_from_parent.obj_name
        att_start = state_init.mode.attachments[obj]
        skeleton[stage_start].guides.append(att_start)
        

    i = 0
    done = False
    while not done:
        if i >= max_iter: break
        
        is_fwd = np.random.choice([True, False])
        # select node to grow
        while True:
            stage = np.random.randint(stage_start, stage_end+1)
            node_curr = skeleton[stage] if is_fwd else skeleton_bwd[stage]
            action = node_curr.action_from_parent
            if node_curr.is_root: continue
            # get random state from parent
            parent_states = node_curr.parent.states
            if len(parent_states) != 0: break

        state_idx = np.random.randint(0, len(parent_states))
        parent_state = parent_states[state_idx]
        
        # get new attachment
        no_guide = len(node_curr.guides) == 0
        if (no_guide) | (0.5 < np.random.random()):
            new_att = domain.sample_instance(
                action.name, action.obj_name, action.parent_to)
        else:
            #guide_idx = np.random.randint(0, len(node_curr.guides))
            #new_att = node_curr.guides[guide_idx]
            guide_idx = np.argmin(node_curr.guides)
            new_att = node_curr.guides.pop(guide_idx)

        # check tool collision and IK feasibility
        q_ik_pre = None
        del_att = parent_state.mode.attachments[action.obj_name]
        if not domain.checker.is_tool_collision(del_att, new_att):
            result = domain.checker.check_edge_ik(del_att, new_att)
            if result is not None:
                q_ik_pre, q_ik, ee_target = result
        if q_ik_pre is None: 
            #new_att.fail -= 1
            continue 

        # connect
        state_new = connect(
            domain, new_att, parent_state, action, q_ik_pre, q_ik, ee_target
        )
        if state_new is None: 
            continue
        i += 1
        # add to tree
        domain.assign(state_new.mode, state_new.config) # for debugging
        state_new.parent = parent_state
        node_curr.states.append(state_new)
        
        
        # check goal
        if (stage == stage_end) & (is_fwd):
            state_fwd = state_new
            state_bwd = skeleton_bwd[stage].states[0]
            node_fwd = node_curr
            
        elif (stage == 0) & (not is_fwd):
            continue
        else:
            #add guide
            node_rev = skeleton_bwd[stage] if is_fwd else skeleton[stage]
            guide_obj = node_rev.action_from_parent.obj_name
            guide_att = state_new.mode.attachments[guide_obj]
            node_rev.guides.append(guide_att)
            #check same mode
            modes_from_rev = [state.mode for state in node_rev.states]
            if not state_new.mode in modes_from_rev: continue
            #finish
            state_rev = [state for state in node_rev.states if state.mode == state_new.mode][0]
            node_fwd = node_curr if is_fwd else node_rev
            state_fwd = state_new if is_fwd else state_rev
            state_bwd = state_rev if is_fwd else state_new
            
        #finish
        traj = connect_in_same_mode(domain, state_fwd, state_bwd, action)
        if traj is None: continue

        trajs = [traj]
        states = []
        #num_bwd = 0
        while True: 
            if state_bwd.parent is None: break
            #num_bwd += 1
            states.append(state_bwd)
            trajs.append(state_bwd.traj_to_parent[::-1])
            state_bwd = state_bwd.parent
        states.append(state_bwd)
        
        parent_state = state_fwd
        for i, stage in enumerate(range(node_fwd.stage+1, stage_end+1)):
            node_new = skeleton[stage]
            mode_new = deepcopy(states[i+1].mode)
            config_new = deepcopy(states[i].config)
            state_new = State(mode_new, config_new)
            state_new.traj_to_parent = trajs[i]
            state_new.parent = parent_state
            node_new.states.append(state_new)
            #update
            parent_state = state_new
        state_new.traj = trajs[-1]
        done = True
        break
    if done:
        domain.assign(state_new.mode, state_new.config)
        return state_new
    return None



def backtrack(last_state_node: State)->List[State]:
    node = last_state_node
    nodes = [last_state_node]
    while True:
        if node.parent is None:
            break
        else:
            node.parent.traj = node.traj_to_parent
        nodes.append(node.parent)
        node = node.parent
    if nodes[0].traj is None:
        nodes[0].traj = [nodes[1].traj[-1]]
    return nodes[::-1]


def connect(
    domain: TAMPDomain, new_att: Attachment, parent_state: State, 
    action: Attach, q_ik_pre: np.ndarray, q_ik: np.ndarray, pose_target: Pose
) -> State:
    if action.name == "pick":
        target_robot = action.parent_to
    elif action.name == "place":
        target_robot = action.parent_from
    config_init = parent_state.config
    #config_goal = deepcopy(parent_state.config)
    state_new = deepcopy(parent_state)
    state_new.mode.set_attachment(new_att)
    
    # ik check
    state_new.config.q[target_robot] = q_ik
    if domain.is_collision(state_new.mode, state_new.config): return None
    state_new.config.q[target_robot] = q_ik_pre

    mp = BiRRT2(q_delta_max=0.05)
    traj = mp.plan([target_robot], config_init, state_new.config, parent_state.mode, domain)
    if traj is None: return None
    domain.assign(parent_state.mode, state_new.config) #debug
    traj_switch = mp.check_mode_switch([target_robot], state_new.config, pose_target, parent_state.mode, domain)
    if traj_switch is not None:
        state_new.traj_to_parent = traj + traj_switch
        #state_new.traj_switch = traj_switch
        state_new.config = traj_switch[-1]
        return state_new
    return None

def connect_in_same_mode(
    domain: TAMPDomain, state_fwd: State, state_bwd: State, fwd_action: Attach
) -> List[Config]:
    if fwd_action.name == "pick":
        target_robot = fwd_action.parent_to
    elif fwd_action.name == "place":
        target_robot = fwd_action.parent_from
    
    mp = BiRRT2()
    traj = mp.plan(
        [target_robot], 
        state_fwd.config, 
        state_bwd.config, 
        state_fwd.mode, domain)
    if traj:        
        return traj
    return None

    
        


if __name__ == "__main__":
    main()