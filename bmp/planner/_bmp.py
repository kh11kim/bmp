#from bmp import *
from pyperplan.task import Operator, Task
from typing import TypeVar

T = TypeVar('T')
@dataclass
class StateNode:
    mode: Mode
    config: Config #q_ik_pre
    index: int # = field(default_factory=lambda :-1) #-1 not assigned
    parent: Optional["StateNode"] = field(default_factory=lambda :None)
    # mode transition
    traj_switch: Optional[List[Config]] = field(default_factory=lambda :None)

    def set_mode_switch_traj(self, traj_switch):
        self.traj_switch = traj_switch #pre -> grasp

@dataclass
class ModeFamilyNode:
    stage: int
    state_disc: frozenset
    index: int
    children: Dict[Operator, "ModeFamilyNode"] = field(default_factory=lambda :{})
    states: Dict[str, List[StateNode]] = field(default_factory=lambda :{"fwd":[], "bwd":[]})
    guides: Dict[str, List[Attachment]] = field(default_factory=lambda :{"fwd":[], "bwd":[]})

    def __repr__(self):
        return f"MF: stage{self.stage} fwd:{len(self.states['fwd'])}, bwd:{len(self.states['bwd'])}, fw_guide:{len(self.guides['fwd'])} bw_guide:{len(self.guides['bwd'])}"

@dataclass
class Trajectory:
    traj: List[Config]
    index: int

class BMPTree:
    def __init__(self):
        self.mf_nodes: List[ModeFamilyNode] = []
        self.state_nodes: List[StateNode] = []
        self.state_edges: List[Trajectory] = [] # motion trajectories
        self.state_edges_dict: Dict[Tuple[int, int], int] = {}

    @property
    def root(self):
        return self.mf_nodes[0]

    def make_state_node(
        self, 
        mode:Mode, 
        config:Config,
        mf:ModeFamilyNode,
        dir:str,
        parent:Optional[StateNode] = None,
        is_root=False
    )->StateNode:  
        index = len(self.state_nodes)
        if not is_root:
            assert parent is not None
        node = StateNode(mode, config, index, parent)
        self.state_nodes += [node]
        mf.states[dir] += [node]
        return node
    
    def make_state_edge(
        self,
        state_from:StateNode,
        state_to:StateNode,
        traj:List[Config],
    ):
        index = len(self.state_edges)
        fwd_traj = Trajectory(traj, index)
        bwd_traj = Trajectory(traj[::-1], index+1)
        self.state_edges += [fwd_traj, bwd_traj]
        self.state_edges_dict[(state_from.index, state_to.index)] = fwd_traj.index
        self.state_edges_dict[(state_to.index, state_from.index)] = bwd_traj.index

    def make_mf_node(
        self,
        stage: int, 
        state_disc: frozenset,
        parent: Optional[ModeFamilyNode] = None,
        is_root=False
    ):
        index = len(self.mf_nodes)
        if not is_root:
            assert parent is not None
        node = ModeFamilyNode(stage, state_disc, index)
        self.mf_nodes += [node]
        return node

    def make_root(
        self, 
        state_disc:frozenset,
        mode:Mode,
        config:Config,
    ):
        root = self.make_mf_node(0, state_disc, is_root=True)
        self.make_state_node(
            mode, config, root, "fwd", is_root=True)

    def is_non_geometric_action(self, action: Operator):
        non_geometric_actions = ["wash", "cook"]
        for action_name in non_geometric_actions:
            if action_name in action.name: 
                return True
        return False

    def get_mode_families(self, action_seqs:List[Operator]):
        node = self.root
        mf_seq = [node]
        for action in action_seqs:
            if self.is_non_geometric_action(action):
                continue

            if action in node.children.keys():
                node_new = node.children[action]
            else:
                #make one
                state_disc = action.apply(node.state_disc)
                node_new = self.make_mf_node(
                    node.stage+1,
                    state_disc, node
                )
                node.children[action] = node_new
            mf_seq += [node_new]
            node = node_new
        return mf_seq

class BMP:
    def __init__(
        self,
        task:Task,
        state_disc:frozenset, 
        mode_init:Mode, 
        config_init:Config,
        domain:TAMPDomain,
        desired_goal_atts: List[Attachment] = [],
    ):
        self.task = task #task object from pyperplan
        self.tree = BMPTree()
        self.tree.make_root(state_disc, mode_init, config_init)
        self.domain = domain
        desired_goal_atts = {att.obj_name:att for att in desired_goal_atts}
        self.desired_goal_atts: Dict[str, Attachment] = desired_goal_atts
    
    def get_attachment_predicates(self, state_disc:frozenset):
        return {pred for pred in state_disc if "attached" in pred}

    

    def get_action(self, mf1:ModeFamilyNode, mf2:ModeFamilyNode):
        mf1_state = mf1.state_disc
        actions: List[Operator] = \
            [action for action in self.task.operators if action.applicable(mf1_state)]
        for action in actions:
            if self.tree.is_non_geometric_action(action):
                continue
            
            state_new = action.apply(mf1_state)
            if self.get_attachment_predicates(state_new) == \
                self.get_attachment_predicates(mf2.state_disc):
                return action
        raise ValueError()

    def sample_goal_state(
        self,
        mf_list:List[ModeFamilyNode],
        max_iter=10
    )->Tuple[Mode, Config]:
        att_preds_init = self.get_attachment_predicates(mf_list[0].state_disc)
        att_preds_goal = self.get_attachment_predicates(mf_list[-1].state_disc)
        unchanged_att_preds = att_preds_init
        for mf in mf_list[1:]:
            unchanged_att_preds = unchanged_att_preds.intersection(mf.state_disc)
        changed_att_preds = att_preds_goal.difference(unchanged_att_preds)
        
        mode_init = mf_list[0].states["fwd"][0].mode
        config_init = mf_list[0].states["fwd"][0].config
        mode_goal = mode_init.copy()
        config_goal = deepcopy(config_init) #assume config_goal == config_init
        for _ in range(max_iter):
            for att_pred in changed_att_preds:
                pred, obj, parent = att_pred.strip("()").split(" ")
                if obj in self.desired_goal_atts:
                    att = deepcopy(self.desired_goal_atts[obj])
                if "robot" in parent:
                    att = self.domain.sample_grasp(obj, parent)
                else:
                    att = self.domain.sample_placement(obj, parent)
                mode_goal.set_attachment(att)
            if not self.domain.is_collision(mode_goal, config_goal):
                return mode_goal, config_goal
        return None

    def set_guides(
        self, 
        mode_init:Mode, mode_goal:Mode, 
        mode_families:List[ModeFamilyNode]):
        
        # fwd
        att_preds = self.get_attachment_predicates(mode_families[0].state_disc)
        for stage, mf in enumerate(mode_families[:-1]):
            mf_next = mode_families[stage+1]
            atts_next = self.get_attachment_predicates(mf_next.state_disc)
            guide_att_pred = att_preds.difference(att_preds.intersection(atts_next))
            if guide_att_pred != set():
                att_pred = list(guide_att_pred)[0]
                pred, obj, parent = att_pred.strip("()").split(" ")
                mode_families[stage].guides["bwd"] += [deepcopy(mode_init.attachments[obj])]
                att_preds -= guide_att_pred
            if att_preds == set(): break

        #bwd
        att_preds = self.get_attachment_predicates(mode_families[-1].state_disc)
        for i, mf in enumerate(mode_families[::-1][:-1]):
            stage = len(mode_families) -1 -i
            mf_next = mode_families[stage-1]
            atts_next = self.get_attachment_predicates(mf_next.state_disc)
            guide_att_pred = att_preds.difference(att_preds.intersection(atts_next))
            if guide_att_pred != set():
                att_pred = list(guide_att_pred)[0]
                pred, obj, parent = att_pred.strip("()").split(" ")
                mode_families[stage].guides["fwd"] += [deepcopy(mode_goal.attachments[obj])]
                att_preds -= guide_att_pred
            if att_preds == set(): break

    def choice(self, a:List[T], out_index=False) -> T:
        idx = np.random.randint(0, len(a))
        if out_index: 
            return a[idx], idx
        return a[idx]

    def get_ik(self, robot_name:str, prev_state:StateNode, new_att:Attachment):
        self.domain.assign(prev_state.mode)
        del_att = prev_state.mode.attachments[new_att.obj_name]
        
        #assume del_att or new_att is a placement.
        placement = del_att if isinstance(del_att, Placement) else new_att
        grasp = del_att if isinstance(del_att, Grasp) else new_att
        parent_pose = self.domain.objects[placement.parent_name].get_base_pose()
        obj_pose = parent_pose * placement.tf.inverse()
        grasp_pose = obj_pose * grasp.tf
        grasp_pose_pre = grasp.get_pre_pose(grasp_pose)
        
        robot = self.domain.robots[robot_name]
        q_pre_ik = robot.inverse_kinematics(pose=grasp_pose_pre)
        if q_pre_ik is None: return None
        config_pre = deepcopy(prev_state.config)
        config_pre.q[robot_name] = q_pre_ik
        mode_new = deepcopy(prev_state.mode)
        mode_new.set_attachment(new_att)
        
        if self.domain.is_collision(mode_new, config_pre): 
            return None
        
        mp = BiRRT2()
        traj_switch = mp.check_mode_switch(
            [robot_name], config_pre, grasp_pose, prev_state.mode, self.domain)
        if traj_switch is not None: 
            return (q_pre_ik, traj_switch)
        return None
        

    def plan(
        self,
        actions: List[Operator],
        domain: TAMPDomain, 
    ):
        self.max_fwd_stage = 0
        mode_families = self.tree.get_mode_families(actions)
        
        #sample goal state and add
        result = self.sample_goal_state(mode_families)
        if result is None: return None
        mode_goal, config_goal = result
        state_goal = self.tree.make_state_node(
            mode_goal, config_goal, 
            mode_families[-1], "bwd", is_root=True)


        #make guides
        mode_init = mode_families[0].states["fwd"][0].mode
        self.set_guides(mode_init, mode_goal, mode_families)
        
        #plan
        max_iter = len(mode_families) -1
        for dir in ["fwd", "bwd"]:
            for i in range(1, max_iter):
                stage = i if dir == "fwd" else len(mode_families) - i - 1               
                rev_dir = "bwd" if dir == "fwd" else "fwd"
                stage_next = stage+1 if dir == "fwd" else stage-1
                stage_prev = stage-1 if dir == "fwd" else stage+1

                node_curr = mode_families[stage]
                node_prev = mode_families[stage_prev] #always exists
                node_next = mode_families[stage_next] if stage_next in range(len(mode_families)) else None
                if len(node_prev.states[dir]) == 0: break
                
                action = self.get_action(node_prev, node_curr)
                rev_action = self.get_action(node_next, node_curr)
                action_name, obj_name, parent_from, parent_to = action.name.strip("()").split(" ")
                _, rev_obj_name, *_ = rev_action.name.strip("()").split(" ")
                robot_name = parent_to if action_name in ["pick", "unstack"] else parent_from
                
                #randomly choose parent_state
                parent_state = self.choice(node_prev.states[dir]) 
                guides = node_curr.guides[dir]
                is_guide = len(guides) != 0
                if is_guide:
                    if dir == "bwd": use_guide = True
                    else:
                        p = np.random.random()
                        use_guide = True if (p < 0.5) else False
                else: use_guide = False
                
                if use_guide:
                    new_att = deepcopy(self.choice(guides))
                else:
                    #sample
                    if action_name in ["pick", "unstack"]:
                        new_att = domain.sample_grasp(obj_name, parent_to)
                    elif action_name in ["place", "stack"]:
                        new_att = domain.sample_placement(obj_name, parent_to)
                mode_new = deepcopy(parent_state.mode)
                mode_new.set_attachment(new_att)

                #if not domain.checker.is_tool_collision(del_att, new_att):
                result = self.get_ik(robot_name, parent_state, new_att)
                if result is None: continue
                q_ik_pre, traj_switch = result
                config_new = deepcopy(parent_state.config)
                config_new.q[robot_name] = q_ik_pre
                
                #connect
                traj = self.connect(robot_name, parent_state, config_new, domain)
                if traj is None: continue

                # make new state node and edge
                state_new = self.tree.make_state_node(
                    mode_new, config_new, node_curr, dir, parent=parent_state)
                state_new.set_mode_switch_traj(traj_switch)
                self.tree.make_state_edge(parent_state, state_new, traj)
                domain.assign(state_new.mode, state_new.config) #debug
                
                #add guide
                if node_next is not None:
                    guide_att = state_new.mode.attachments[rev_obj_name]
                    node_curr.guides[rev_dir].append(guide_att)
                
                if dir == "fwd": self.max_fwd_stage = stage
                
                # goal check
                if (dir == "fwd") & (node_next is None):
                    goal_att_satisfied = True
                    for goal_att in self.desired_goal_atts:
                        if goal_att not in mode_new.attachments:
                            goal_att_satisfied = False
                    if not goal_att_satisfied: continue
                    print("fwd tree is reaching the goal mode")
                    state_fwd, state_bwd = state_new, mode_families[-1].states["bwd"][0]
                elif use_guide:
                    is_two_tree_meet = False
                    for state in node_curr.states[rev_dir]:
                        if mode_new == state.mode:
                            is_two_tree_meet = True
                    if not is_two_tree_meet: continue
                    print("two tree meet in the middle")
                    state_fwd, state_bwd = state_new, state
                    if dir == "bwd": 
                        state_fwd, state_bwd = state_bwd, state_fwd
                    else:continue
                else: continue
                
                # connect last states in mode
                traj_connect = self.connect_in_the_same_mode(
                    robot_name,
                    state_fwd, state_bwd, domain)
                if traj_connect is not None:
                    print("finish")
                    self.tree.make_state_edge(state_fwd, state_bwd, traj_connect)
                    return True#self.postprocess(state_fwd, state_bwd, domain)
            #max_iter = max_fwd_stage
        return None
    
    def postprocess(self, state_fwd, state_bwd, domain):
        states_fwd = self.backtrack(state_fwd)
        states_bwd = self.backtrack(state_bwd)[::-1]
        
        _states = states_fwd+states_bwd
        states = states_fwd + states_bwd[1:]
        trajs = []
        for i, _ in enumerate(_states[:-1]):
            mp = BiRRT2()
            mp.init(["robot"], domain, states[i].mode)
            parent = _states[i]
            child = _states[i+1]
            traj_idx = self.tree.state_edges_dict[(parent.index, child.index)]
            traj1 = [] if parent.traj_switch is None else parent.traj_switch[::-1]
            traj2 = self.tree.state_edges[traj_idx].traj
            traj3 = [] if child.traj_switch is None else child.traj_switch
            trajs.append(mp.smoothing(traj1+traj2+traj3))
        
        return states, trajs

    def backtrack(self, state:StateNode) -> List[StateNode]:
        nodes = [state]
        node = state
        while True:
            if node.parent is None: 
                return nodes[::-1]
            node = node.parent
            nodes.append(node)

    def connect(
        self,
        robot_name: str,
        parent_state: StateNode,
        config_new: Config,
        domain: TAMPDomain
    ):
        mp = BiRRT2(q_delta_max=0.1, ts_eps=0.01)
        traj = mp.plan([robot_name], parent_state.config, config_new, parent_state.mode, domain)
        if traj is not None: 
            return traj
        return None
        

    def connect_in_the_same_mode(
        self, robot_name:str,
        state_fwd: StateNode, state_bwd: StateNode, domain:TAMPDomain
    ) -> List[Config]:
        traj_switch1, traj_switch2 = [], []
        if state_fwd.traj_switch is not None: traj_switch1 = state_fwd.traj_switch[::-1]
        if state_bwd.traj_switch is not None: traj_switch2 = state_bwd.traj_switch
        
        mp = BiRRT2()
        traj = mp.plan([robot_name], 
            state_fwd.config, state_bwd.config, 
            state_fwd.mode, domain)
        if traj: 
            return traj_switch1 + traj + traj_switch2
        return None