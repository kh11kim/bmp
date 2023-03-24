from bmp.base import *
from bmp.planner.rrt import *
from dataclasses import dataclass
from typing import TypeVar
T = TypeVar("T")

# @dataclass
# class StateNode:
#     mode: Mode
#     config: Config #q_ik_pre
#     stage: int
#     direction: str = field(default_factory=lambda :"")
#     parent: Optional["StateNode"] = field(default_factory=lambda :None)
#     q_ik: Optional[np.ndarray] = field(default_factory=lambda :None)
#     ee_pose: Optional[Pose] = field(default_factory=lambda :None)
#     index: int = field(default_factory=lambda :-1)
#     traj_switch: Optional[List[Config]] = field(default_factory=lambda :None)
#     #parent: Optional["StateNode"] = field(default_factory=lambda :None)
#     #traj_to_parent: Optional[List[Config]] = field(default_factory=lambda :None)
#     #traj_switch: Optional[List[Config]] = field(default_factory=lambda :None)
#     #traj: Optional[List[Config]] = field(default_factory=lambda :None)

#     def copy(self):
#         mode = deepcopy(self.mode)
#         config = deepcopy(self.config)
#         return StateNode(mode, config, self.stage, self.direction)

# @dataclass
# class StateEdge:
#     parent: StateNode
#     child: StateNode
#     traj: Optional[List[Config]] = field(default_factory=lambda :None)

# @dataclass
# class ModeFamily:
#     vars: Dict[str, str] #obj, parent
#     nums: Dict[str, int] = field(default_factory=lambda : {}) #obj, att_index

#     @classmethod
#     def from_mode(cls, mode:Mode, numbers=None):
#         var = {}
#         num = {}
#         for obj, (parent, _) in mode.mode_key.items():
#             var[obj] = parent
#         return cls(var)
    
#     def as_string(self, numbered=False):
#         s = []
#         for obj, parent in self.vars.items():
#             if numbered:
#                 num = self.nums[obj]
#                 s.append(f"{obj}-{parent}-{num}")
#             else:
#                 s.append(f"{obj}-{parent}")
#         return "_".join(s)
    
#     def get_new_mf_by_transition(self, obj:str, parent:str)->"ModeFamily":
#         vars = deepcopy(self.vars)
#         nums = deepcopy(self.nums)
#         vars[obj] = parent
#         return self.__class__(vars, nums)
    
#     def get_variables(self, objs:List[str]=None):
#         if objs is None:
#             objs = self.vars.keys() #all
#         result = []
#         for obj in objs:
#             parent = self.vars[obj]
#             num = self.nums[obj]
#             result.append(f"{obj}-{parent}-{num}")
#         return result

#     def __repr__(self):
#         return f"MF: {self.as_string(numbered=True)}"

# @dataclass
# class ModeFamilyNode:
#     stage: int
#     mode_family: ModeFamily
#     index: int
#     # is_fwd, state_list
#     states: Dict[str, List[StateNode]] = field(default_factory=lambda :{"fwd":[], "bwd":[]})
#     guides: Dict[str, List[Attachment]] = field(default_factory=lambda :{"fwd":[], "bwd":[]})
#     is_root: bool = field(default_factory=lambda :False)
#     is_terminal: bool = field(default_factory=lambda :False)


#     def __repr__(self):
#         if self.is_terminal:
#             return "terminal"
#         else:
#             return f"stage{self.stage}_{self.mode_family.as_string(numbered=True)} fwd:{len(self.states['fwd'])}, bwd:{len(self.states['bwd'])}"

    # def sampling_var(self):
    #     obj = self.prev_action.obj_name
    #     return self.mode_family.get_variables(objs=[obj])[0]
    
    # def find_state_by_mode(self, mode_ref:Mode):
    #     np.random.shuffle(self.states)
    #     for state in self.states:
    #         if state.mode == mode_ref:
    #             return state
    #     return None

@dataclass
class ModeFamilyEdge:
    parent: ModeFamilyNode
    child: ModeFamilyNode
    is_terminal: bool = field(default_factory=False)
    action: Optional[Attach] = field(default_factory=lambda : None)

    # @classmethod
    # def from_attach(cls, parent:ModeFamilyNode, child:ModeFamilyNode, attach:Attach):
    #     return cls(
    #         obj_name=attach.obj_name,
    #         parent_from=attach.parent_from,
    #         parent_to=attach.parent_to,
    #         name=attach.name,
    #         parent=parent, child=child
    #     )


class ModeFamilyTree:
    def __init__(self, state_init:StateNode):
        self.mf_nodes: List[ModeFamilyNode] = [] #Dict[int, List[ModeFamilyNode]] = {0:[]}
        self.state_nodes: List[StateNode] = []
        # (idx, idx) -> Edge
        self.mf_edges: Dict[int, Dict[str, ModeFamilyEdge]] = {} 
        # (idx, idx) -> Edge
        self.state_edges: Dict[Tuple[Tuple[int, int]], StateEdge] = {}
        self.variables = []

        mf_root = ModeFamily.from_mode(state_init.mode)
        for obj_name in mf_root.vars.keys():
            self.assign_new_var_number(mf_root, obj_name)
        self.make_mf_node(0, mf_root, is_root=True)
        state_init.direction = "fwd"
        self.set_state_node(state_init, self.root)
        #self.root.states["fwd"].append(state_init)

        #self.add_mf_node_edge(0, mf_root)
        # root = ModeFamilyNode(0, mf_root, is_root=True)
        # root.states.append(state_init)
        #self.mf_node[0].append(root)
    
    @property
    def root(self)->ModeFamilyNode:
        return self.mf_nodes[0]

    def set_state_node(
        self, 
        state_new: StateNode,
        mf_node_to_add:Optional[ModeFamilyNode]=None, 
    ):
        index = len(self.state_nodes)
        state_new.index = index
        self.state_nodes.append(state_new)
        mf_node_to_add.states[state_new.direction].append(state_new)
        return state_new
    
    def set_state_edge(
        self, 
        parent:StateNode, 
        new_node:StateNode, 
        traj:List[Config],
        set_parent:bool = True
    ):
        if set_parent:
            new_node.parent = parent
        edge = StateEdge(parent, new_node, traj)
        self.state_edges[(parent.index, new_node.index)] = edge
        edge = StateEdge(new_node, parent, traj[::-1])
        self.state_edges[(new_node.index, parent.index)] = edge
        

    def make_mf_node(self, stage:int, mf: Mode, is_root=False, is_terminal=False):
        node_index = len(self.mf_nodes)
        mf_node = ModeFamilyNode(stage, mf, node_index, is_root=is_root, is_terminal=is_terminal)
        self.mf_nodes.append(mf_node)
        self.mf_edges[node_index] = {}
        return mf_node
    
    def set_mf_edge(
        self, 
        parent:ModeFamilyNode, 
        child:ModeFamilyNode,
        is_terminal=False,
        attach:Optional[Attach]=None,
        #dir:str = "fwd"
    ):
        if parent.index not in self.mf_edges:
            self.mf_edges[parent.index] = {}
        
        if is_terminal:
            mf_edge = ModeFamilyEdge(parent, child, is_terminal, attach)
            self.mf_edges[parent.index]["fwd_terminal"] = mf_edge
        else:
            mf_edge = ModeFamilyEdge(parent, child, is_terminal, attach)
            fwd_key = f"fwd_{child.mode_family.as_string()}"
            self.mf_edges[parent.index][fwd_key] = mf_edge
            mf_edge = ModeFamilyEdge(child, parent, is_terminal, attach.reverse())
            bwd_key = f"bwd_{parent.mode_family.as_string()}"
            self.mf_edges[child.index][bwd_key] = mf_edge
        return mf_edge

    def get_mf_edge(
        self, 
        parent:ModeFamilyNode, 
        child:ModeFamilyNode, 
        dir:str="fwd"
    ):
        key = f"{dir}_{child.mode_family.as_string()}"
        return self.mf_edges[parent.index][key]

    def assign_new_var_number(self, mf: ModeFamily, obj:str):
        parent, num = mf.vars[obj], 0
        while True:
            if f"{obj}-{parent}-{num}" not in self.variables: break
            num += 1
        new_var = f"{obj}-{parent}-{num}"
        self.variables.append(new_var)
        mf.nums[obj] = num

    # def make_bwd_skeleton(self, fwd_skeleton: List[ModeFamilyNode]):
    #     #bwd_skeleton: List[ModeFamilyNode] = []
    #     bwd_root = bw
    #     for stage, fwd_node in enumerate(fwd_skeleton):
    #         bwd_node = ModeFamilyNode(stage, fwd_node.mode_family, index=0)
    #         bwd_skeleton.append(bwd_node)
    #     for stage, bwd_node in enumerate(bwd_skeleton[:-1]):
    #         bwd_node.parent = bwd_skeleton[stage+1]
    #         bwd_node.prev_action = fwd_skeleton[stage+1].prev_action.reverse()
    #     bwd_skeleton[-1].is_root = True
    #     return bwd_skeleton
    
    # def add_node(self, node:ModeFamilyNode, parent:ModeFamilyNode):
    #     mf_str = node.mode_family.as_string()
    #     node.parent = parent
    #     if node.stage not in self.mf_node:
    #         self.mf_node[node.stage] = []
    #     self.mf_node[node.stage].append(node)
    #     parent.childs[mf_str] = node

    def add_mode_families_by_action_seq(self, actions:List[Attach]):
        parent = self.root
        mf_list = [self.root.mode_family]
        for stage, a in enumerate(actions, start=1):
            obj_name, parent_obj_name = a.obj_name, a.parent_to
            mf_new = parent.mode_family.get_new_mf_by_transition(obj_name, parent_obj_name)
            if mf_new.as_string() in self.mf_edges[parent.index]:
                edge = self.mf_edges[parent.index][mf_new.as_string()]
                node_new = edge.child
            else:
                self.assign_new_var_number(mf_new, obj_name)
                node_new = self.make_mf_node(stage, mf_new)
                edge = self.set_mf_edge(parent, node_new, attach=a)
            mf_list.append(node_new.mode_family)
            parent = node_new

        #terminal
        terminal_node = self.make_mf_node(stage, mf_new, is_terminal=True)
        self.set_mf_edge(node_new, terminal_node, is_terminal=True)

        # # backward node-edge
        # parent = bwd_root
        # for i, a in enumerate(actions[::-1]):
        #     stage = len(actions) - i - 1
        #     mf_bwd = mf_list[stage]
        #     node_new = self.make_mf_node(stage, mf_bwd, is_fwd=False)
        #     edge = self.set_mf_edge(parent, node_new, a.reverse())
        #     parent = node_new
            
    
    def get_skeleton(self):
        skeleton = [self.root]
        node = self.root
        while True:
            fwd_key = [key for key in self.mf_edges[node.index].keys() if "fwd" in key]
            key = choice(fwd_key)
            if "terminal" in key:
                break
            node = self.mf_edges[node.index][key].child
            skeleton.append(node)
        #bwd_skeleton = self.mf_edges[node.index]["terminal"]
        return skeleton #, bwd_skeleton

    # def get_skeleton_by_action(self, actions):
    #     skeleton = [self.root]
    #     node = self.root
    #     for action in actions:

    #     while True:
    #         fwd_key = [key for key in self.mf_edges[node.index].keys() if "fwd" in key]
    #         key = choice(fwd_key)
    #         if "terminal" in key:
    #             break
    #         node = self.mf_edges[node.index][key].child
    #         skeleton.append(node)
    #     #bwd_skeleton = self.mf_edges[node.index]["terminal"]
    #     return skeleton #, bwd_skeleton

def choice(a:List[T], out_index=False) -> T:
    idx = np.random.randint(0, len(a))
    if out_index:
        return a[idx], idx
    return a[idx]
    
class BMP:
    def __init__(self):
        pass
    
    def sample_goal_state(
        self, 
        state_init:StateNode,
        mf_init:ModeFamily, 
        mf_goal:ModeFamily,
        domain:TAMPDomain,
        max_iter=10
    ):
        init_variables = set(mf_init.get_variables())
        goal_variables = set(mf_goal.get_variables())
        changed_variables = goal_variables.difference(init_variables)
        state_goal = deepcopy(state_init)
        state_goal.is_fwd = False
        
        for _ in range(max_iter):
            for variable in changed_variables:
                obj, parent, _ = variable.split("-")
                if "robot" in parent:
                    att = domain.sample_grasp(obj, parent)
                else:
                    att = domain.sample_placement(obj, parent)
                state_goal.mode.set_attachment(att)
            if not domain.is_collision(state_goal.mode, state_goal.config):
                return state_goal
        return None

    def plan(
        self,
        mftree: ModeFamilyTree,
        skeleton: List[ModeFamilyNode], 
        domain: TAMPDomain, 
    ):
        max_fwd_stage = 0
        #sample goal state and add
        state_init: StateNode = skeleton[0].states["fwd"][0]
        mf_init = skeleton[0].mode_family
        mf_goal = skeleton[-1].mode_family
        state_goal_sampled = self.sample_goal_state(
            state_init, mf_init, mf_goal, domain)
        if state_goal_sampled is None: return max_fwd_stage
        state_goal_sampled.direction = "bwd"
        mftree.set_state_node(state_goal_sampled, skeleton[-1])
        
        #make guides
        known_atts = {}
        init_vars = skeleton[0].mode_family.get_variables()
        for var in init_vars:
            obj, *_ = var.split("-")
            known_atts[var] = state_init.mode.attachments[obj]
        goal_vars = skeleton[-1].mode_family.get_variables()
        for var in goal_vars:
            obj, *_ = var.split("-")
            known_atts[var] = state_goal_sampled.mode.attachments[obj]
        
        known_vars = set(known_atts.keys())
        #state_guide = deepcopy(state_init)
        #state_guide.direction = "guide"
        for dir in ["fwd", "bwd"]:
            start, end = 1, len(skeleton)
            for i in range(start, end):
                node1, node2 = skeleton[i-1], skeleton[i]
                if dir == "bwd": node1, node2 = node2, node1
                action = mftree.get_mf_edge(node1, node2, dir).action
                sampling_var = node2.mode_family.get_variables(objs=[action.obj_name])[0]
                if sampling_var in known_vars:
                    att = deepcopy(known_atts[sampling_var])
                    node2.guides[dir].append(att)
            
            #vars = set(node.mode_family.get_variables())
            #if vars.difference(known_vars) == set():
                

        #plan
        #fwd_iter = bwd_iter = 0
        max_iter = len(skeleton)-1
        for dir in ["fwd", "bwd"]:
            for i in range(max_iter):
                #dir = "fwd"
                #dir = "fwd" if fwd_iter <= bwd_iter else "bwd"
                #dir = "fwd"
                #node_curr, stage = choice(skeleton, out_index=True)
                if dir == "fwd": stage = i + 1
                elif dir == "bwd": stage = len(skeleton) - i - 2
                rev_dir = "bwd" if dir == "fwd" else "fwd"
                stage_next = stage+1 if dir == "fwd" else stage-1
                stage_prev = stage-1 if dir == "fwd" else stage+1

                node_curr = skeleton[stage]
                node_prev = skeleton[stage_prev] #always exists
                node_next = skeleton[stage_next] if stage_next in range(len(skeleton)) else None
                
                if len(node_prev.states[dir]) == 0: continue
                #if (stage, dir) in [(0, "bwd")]: continue
                
                parent_state = choice(node_prev.states[dir])
                action = mftree.get_mf_edge(node_prev, node_curr, dir).action
                del_att = parent_state.mode.attachments[action.obj_name]
                #node_next = skeleton[stage_next]
                #guide_states = node_next.states[rev_dir] + node_next.states["guide"]
                
                guides = node_curr.guides[dir]
                is_guide = len(guides) != 0
                if is_guide:
                    if dir == "bwd": use_guide = True
                    else:
                        p = np.random.random()
                        use_guide = True if (p < 0.5) else False
                else: use_guide = False
                
                #use_guide = False
                if use_guide:
                    new_att = deepcopy(choice(guides))
                    #guide_state = choice(guide_states)
                    #deepcopy(guide_state.mode.attachments[action.obj_name])
                else:
                    new_att = domain.sample_instance_by_action(action)
                mode_new = deepcopy(parent_state.mode)
                mode_new.set_attachment(new_att)

                # make new state
                result = None
                if not domain.checker.is_tool_collision(del_att, new_att):
                    result = domain.get_ik(action.robot_name, parent_state.mode, del_att, new_att)
                    if result is None: continue
                    q_ik_pre, q_ik, ee_target = result
                else: continue
                config_new = deepcopy(parent_state.config)
                config_new.q[action.robot_name] = q_ik_pre
                state_new = StateNode(
                    mode_new, config_new, stage_next, direction=dir, q_ik=q_ik, ee_pose=ee_target)
                

                #connect
                result = self.connect(action.robot_name, parent_state, state_new, domain)
                if result is None: continue
                state_new, traj = result
                domain.assign(state_new.mode, state_new.config) #debug
                # if dir == "fwd": fwd_iter += 1
                # else: bwd_iter += 1
                mftree.set_state_node(state_new, node_curr)
                mftree.set_state_edge(parent_state, state_new, traj)
                
                #add guide
                if node_next is not None:
                    rev_action = mftree.get_mf_edge(node_next, node_curr, rev_dir).action
                    guide = state_new.mode.attachments[rev_action.obj_name]
                    node_curr.guides[rev_dir].append(guide)
                
                if dir == "fwd":
                    max_fwd_stage = stage
                # goal check
                if (dir == "fwd") & (stage == len(skeleton)-1):#id(node_curr) == id(skeleton[-1]):
                    print("fwd tree is reaching the goal mode")
                    state_fwd, state_bwd = state_new, skeleton[-1].states["bwd"][0]
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
                    action.robot_name,
                    state_fwd, state_bwd, domain)
                if traj_connect is not None:
                    print("finish")
                    mftree.set_state_edge(state_fwd, state_bwd, traj_connect, set_parent=False)
                    return self.postprocess(mftree, state_fwd, state_bwd, domain)
            #max_iter = max_fwd_stage
        return max_fwd_stage
    
    def backtrack(self, state:StateNode) -> List[StateNode]:
        nodes = [state]
        node = state
        while True:
            if node.parent is None: 
                return nodes[::-1]
            node = node.parent
            nodes.append(node)

    def postprocess(
        self, 
        mftree:ModeFamilyTree,
        state_fwd: StateNode, 
        state_bwd:StateNode,
        domain: TAMPDomain
    ):
        #states:List[StateNode] = []
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
            traj = mftree.state_edges[(parent.index, child.index)].traj
            trajs.append(mp.smoothing(traj))
        
        return states, trajs


    def connect(
        self,
        robot_name: str,
        parent_state: StateNode,
        state_new: StateNode,
        domain: TAMPDomain
    ):
        mp = BiRRT2(q_delta_max=0.1, ts_eps=0.01)
        if parent_state.traj_switch is not None:
            traj_switch1 = parent_state.traj_switch[::-1]
        else: traj_switch1 = []

        traj_switch2 = mp.check_mode_switch(
            [robot_name], state_new.config, state_new.ee_pose, state_new.mode, domain)
        if traj_switch2 is None: return None

        traj = mp.plan([robot_name], parent_state.config, state_new.config, parent_state.mode, domain)
        if traj is None: return None
        #domain.assign(parent_state.mode, state_new.config) #debug
        state_new.traj_switch = traj_switch2
        traj = traj_switch1 + traj + traj_switch2
        return state_new, traj

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

    # def make_bwd_skeleton(self, fwd_skeleton: List[ModeFamilyNode]):
    #     bwd_skeleton: List[ModeFamilyNode] = []
    #     for stage, fwd_node in enumerate(fwd_skeleton):
    #         bwd_node = ModeFamilyNode(stage, fwd_node.mode_family, index=0)
    #         bwd_skeleton.append(bwd_node)
    #     for stage, bwd_node in enumerate(bwd_skeleton[:-1]):
    #         bwd_node.parent = bwd_skeleton[stage+1]
    #         bwd_node.prev_action = fwd_skeleton[stage+1].prev_action.reverse()
    #     bwd_skeleton[-1].is_root = True
    #     return bwd_skeleton

# def backtrack(last_state_node: StateNode)->List[StateNode]:
#     node = last_state_node
#     nodes = [last_state_node]
#     while True:
#         if node.parent is None:
#             break
#         else:
#             node.parent.traj = node.traj_to_parent
#         nodes.append(node.parent)
#         node = node.parent
#     if nodes[0].traj is None:
#         nodes[0].traj = [nodes[1].traj[-1]]
#     return nodes[::-1]


if __name__ == "__main__":
    from bmp.base import *
    from bmp.domain.kitchen import *
    from time import sleep

    dom = DomainKitchen(gui=True, num_box=2)
    prob = ProblemKitchen(dom, num_block=2, cooked_list=[0,1])
    
    actions = [
        Pick("box1", "dish1", "robot"),
        Place("box1", "robot", "sink1"),
        Pick("box1", "sink1", "robot"),
        Place("box1", "robot", "oven1"),
        Pick("box0", "dish1", "robot"),
        Place("box0", "robot", "sink1"),
        Pick("box0", "sink1", "robot"),
        Place("box0", "robot", "oven1"),
    ]
    actions2 = [
        Pick("box1", "dish1", "robot"),
        Place("box1", "robot", "sink1"),
        Pick("box0", "dish1", "robot"),
        Place("box0", "robot", "sink1"),
        Pick("box0", "sink1", "robot"),
        Place("box0", "robot", "oven1"),
        Pick("box1", "sink1", "robot"),
        Place("box1", "robot", "oven1"),
    ]

    #make mode family nodes from action sequence
    state_init = StateNode(prob.mode_init, prob.config_init)
    mftree = ModeFamilyTree(state_init)
    mftree.add_mode_families_by_action_seq(actions)
    mftree.add_mode_families_by_action_seq(actions2)
    fwd_skeleton, bwd_skeleton = mftree.get_skeleton()

    bmp = BMP()
    last_state = bmp.plan(fwd_skeleton, bwd_skeleton, dom, max_iter=100)
    #bmp(fwd_skeleton, bwd_skeleton, dom, max_iter=100)
    #if last_state is not None:
        #states = backtrack(last_state)
        # for state in states:
        #     mp = BiRRT2()
        #     mp.init(["robot"], dom, state.mode)
        #     for config in mp.smoothing(state.traj):
        #         dom.assign(state.mode, config)
        #         sleep(0.1)

    input()