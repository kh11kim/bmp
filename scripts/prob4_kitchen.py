from pybullet_suite import *
from copy import deepcopy
from typing import Optional, List, Dict, Set
from dataclasses import dataclass, field
import numpy as np
from bmf import *

@dataclass
class State:
    mode: Mode
    config: Config
    parent: Optional["State"] = field(default_factory=lambda :None)
    traj_to_parent: Optional[List[Config]] = field(default_factory=lambda :None)
    traj: Optional[List[Config]] = field(default_factory=lambda :None)

@dataclass
class ModeFamily:
    stage: int
    mode_family_str: str
    states: List[State] = field(default_factory=lambda :[])
    parent: Optional["ModeFamily"] = field(default_factory=lambda :None)
    action_from_parent: Optional[Attach] = field(default_factory=lambda :None)
    childs: Dict[str,"ModeFamily"] = field(default_factory=lambda :{})

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
    
    def get_var(self):
        return self.numbered_var_dict[self.action_from_parent.obj_name]
    

class ModeFamilyTree:
    def __init__(self):
        self.V: Dict[int, ModeFamily] = {0:[]}
        self.variables = []


    def get_key(self, var):
        # var (str): obj-parent
        num = 0
        while True:
            key = f"{var}-{num}"
            if key not in self.variables:
                break
            num += 1
        return key
    
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
            same_vars = set(prev_var_dict.values()).intersection(new_var_dict.values())
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
    
    def parse_mode_families_by_action(self, mode_init: Mode, actions: List[Attach]):
        # init
        if len(self.V[0]) == 0:    
            var_dict = {}
            for obj, att in mode_init.attachments.items():
                parent = att.parent_name
                var_dict[obj]= f"{obj}-{parent}"
            root: ModeFamily = self.add_mode_family(var_dict, is_root=True)
        else: 
            root = self.V[0][0]
            var_dict = root.var_dict

        node = root
        nodes = [root]
        for action in actions:
            obj, parent = action.obj_name, action.parent_to
            var_dict[obj] = f"{obj}-{parent}"
            var_str = self.make_var_str(var_dict)
            if var_str not in node.childs:
                node = self.add_mode_family(var_dict, node)
                node.action_from_parent = action
            else:
                node = node.childs[var_str]
            nodes.append(node)
        return nodes

    def make_var_str(self, var_dict:Dict):
        variables = []
        for obj, var in var_dict.items():
            variables.append(var)
        return "_".join(variables)

def main():
    domain = DomainKitchen(gui=True)
    mode_init = domain.init_mode
    config_init = domain.init_config

    actions = [
        Pick(obj_name="box1", parent_from="dish", parent_to="robot"),
        Place(obj_name="box1", parent_from="robot", parent_to="oven"),
        Pick(obj_name="box2", parent_from="dish", parent_to="robot"),
        Place(obj_name="box2", parent_from="robot", parent_to="oven"),
        Pick(obj_name="box3", parent_from="dish", parent_to="robot"),
        Place(obj_name="box3", parent_from="robot", parent_to="oven"),
        Pick(obj_name="box4", parent_from="dish", parent_to="robot"),
        Place(obj_name="box4", parent_from="robot", parent_to="oven"),
    ]
    
    #mode_family_tree = {}
    mf_tree = ModeFamilyTree()
    nodes = mf_tree.parse_mode_families_by_action(mode_init, actions)
    mf_tree.V[0][0].states.append(State(mode_init, config_init))
    
    #TODO: go to tree
    for _ in range(1000):
        stage = np.random.randint(1, len(nodes))
        node_curr = nodes[stage]
        action = node_curr.action_from_parent
        var = node_curr.get_var()

        # get parent
        parent_states = node_curr.parent.states
        if len(parent_states) == 0: continue
        state_idx = np.random.randint(0, len(parent_states))
        parent_state = parent_states[state_idx]
        
        # get new attachment
        new_att = domain.sample_instance(
            action.name, action.obj_name, action.parent_to)

        # check
        q_ik_pre = None
        del_att = parent_state.mode.attachments[action.obj_name]
        if not domain.checker.is_tool_collision(del_att, new_att):
            result = domain.checker.check_edge_ik(del_att, new_att)
            if result is not None:
                q_ik_pre, ee_pose = result
        if q_ik_pre is None: continue 

        result = connect(
            domain, new_att, parent_state, action, q_ik_pre, ee_pose
        )
        if result is None: continue
        state_new, traj = result
        domain.assign(state_new.mode, state_new.config) #debug
        state_new.parent = parent_state
        state_new.traj_to_parent = traj
        state_new.traj = [traj[-1]]
        nodes[stage].states.append(state_new)
        if stage == len(nodes)-1:
            print("finish")
            break
    
    states = backtrack(state_new)
    for state in states:
        for config in state.traj:
            domain.assign(state.mode, config)
    input()

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
    return nodes[::-1]


def connect(
    domain: TAMPDomain, new_att: Attachment, parent_state: State, 
    action: Attach, q_ik_pre: np.ndarray, ee_pose: Pose
) -> Optional[Tuple[State, List[Config]]]:
    if action.name == "pick":
        target_robot = action.parent_to
    elif action.name == "place":
        target_robot = action.parent_from
    config_init = parent_state.config
    config_goal = deepcopy(config_init)
    config_goal.q[target_robot] = q_ik_pre

    atts = parent_state.mode.attachments #copy
    atts[new_att.obj_name] = new_att
    mode_new = Mode(atts)
    if domain.is_collision(mode_new, config_goal):
        return None

    mp = BiRRT2()
    traj = mp.plan([target_robot], config_init, config_goal, parent_state.mode, domain)
    if traj:
        # TODO: check mode switch
        return State(mode_new, config_goal), traj
    return None


    
        


if __name__ == "__main__":
    main()