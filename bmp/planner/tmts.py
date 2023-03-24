from bmp import *
from pyperplan.task import Operator, Task
from pyperplan.planner import HEURISTICS, SEARCHES, _parse, _ground, _search
from pyperplan.pddl.parser import Parser
from typing import TypeVar
from dataclasses import dataclass
import time

T = TypeVar('T')

def choose_random(x:List[T])->T:
    i = np.random.randint(0, len(x))
    return x[i]

def split_att_str(att:str):
    _, obj, parent = att.strip("()").split(" ")
    return obj, parent

@dataclass
class TaskNode:
    state: frozenset
    actions: List[Operator]
    parent: Optional["TaskNode"] = field(default_factory=lambda :None)
    children: Dict[Operator,"TaskNode"] = field(default_factory=lambda :{})
    visit: int = field(default_factory=lambda :0)
    sum_reward: int = field(default_factory=lambda :0)
    is_terminal: bool = field(default_factory=lambda : False)
    index: int = field(default_factory=lambda :-1)
    
    @property
    def fully_expanded(self):
        return len(self.actions) == len(self.children.keys())

@dataclass
class TaskNodeTerminal(TaskNode):
    state: Optional[frozenset] = field(default_factory=lambda :None)
    actions: Optional[List] = field(default_factory=lambda :[])
    is_terminal: bool = field(default_factory=lambda :True)

class TaskTree:
    def __init__(self, domain_file:str, problem_file:str):
        self.domain_file = domain_file 
        self.problem_file = problem_file
        
        self.parser = Parser(
            self.domain_file, probFile=self.problem_file)
        self.domain = self.parser.parse_domain()
        self.problem = self.parser.parse_problem(self.domain)
        self.task = _ground(self.problem)

        self.search = SEARCHES["wastar"]
        heuristic_class = HEURISTICS["hff"]
        self.heuristic = heuristic_class(self.task)

        #root
        self.state_init = self.task.initial_state
        actions = self.get_applicable_actions(self.state_init)
        self.V: List[TaskNode] = [TaskNode(self.state_init, actions, index=0)]
    
    @property
    def root(self):
        return self.V[0]
    
    def add_child(self, parent:TaskNode, action:Union[Operator, str]):
        index = len(self.V)
        if isinstance(action, str): #terminal
            child = TaskNodeTerminal(parent=parent, index=index)
        else:
            state_new = action.apply(parent.state)
            actions = self.get_applicable_actions(state_new)
            child = TaskNode(state_new, actions, parent=parent, index=index)
        parent.children[action] = child
        self.V += [child]
        return child

    def get_applicable_actions(self, state:frozenset):
        return [op for op in self.task.operators if op.applicable(state)]
    
    def get_task_plan(self, state:frozenset):
        self.task.initial_state = state
        return _search(self.task, self.search, self.heuristic)

class TaskPlanner:
    def __init__(
        self, 
        domain_file:str, 
        problem_file:str,
        alpha:float=0.5,#pw parameter
        eps:float=0.8, #epsilon greedy parameter
        c:float=1, #uct parameter
    ):
        self.task_tree = TaskTree(domain_file, problem_file)
        self.state_init = self.task_tree.state_init
        self.alpha = alpha
        self.eps = eps
        self.c = c
        self.p_terminal = 0.05
    
    def sample_action_seq(self):
        node, pi_tree = self.task_tree_policy2()
        pi_plan = self.get_task_plan(node.state)
        pi = pi_tree + pi_plan
        self.set_tree_by_pi(pi)
        return pi_tree + pi_plan
    
    def set_tree_by_pi(self, pi:List[Operator]):
        node = self.task_tree.root
        for a in pi:
            if a in node.children:
                node = node.children[a]
            else:
                node = self.task_tree.add_child(node, a)

    def get_task_plan(self, state:frozenset):
        return self.task_tree.get_task_plan(state)

    # def task_tree_policy(self):
    #     pi_tree = []
    #     node = self.task_tree.root
    #     # selection
    #     while True:
    #         #node.visit += 1
    #         if node.is_terminal:
    #             break
    #         elif (len(node.children) == 0): # or (self.pw_test(node))
    #             #print("new_task_plan")
    #             node_new, action = self.expand_new_child(node)
    #             node_new.visit += 1
    #             pi_tree += [action]
    #             node = self.make_terminal_node(node_new)
    #         else:
    #             #node, action = self.select_by_uct(node)
    #             node, action = self.select_by_egreedy(node)
    #             if not node.is_terminal: pi_tree += [action]

    #     node_last = node.parent
    #     return (node_last, pi_tree)
    
    def task_tree_policy2(self):
        pi_tree = []
        node = self.task_tree.root
        # selection
        while True:
            if (len(node.children) == 0):
                a = choose_random(node.actions)
                node = self.task_tree.add_child(node, a)
                pi_tree += [a]
                break
            else:
                a = self.select_by_egreedy2(node)
                if isinstance(a, str):
                    break
                elif a not in node.children:
                    #create new node and break
                    node = self.task_tree.add_child(node, a)
                    pi_tree += [a]
                    break
                else:
                    #select the node and continue
                    node = node.children[a]
                    pi_tree += [a]
        return (node, pi_tree)
    
    def pw_test(self, node: TaskNode):
        if node.fully_expanded: 
            return False
        left = (node.visit)**self.alpha
        right = (node.visit-1)**self.alpha
        result = np.floor(left) > np.floor(right)
        #print(f"{result}")
        return result

    # def expand_new_child(self, node: TaskNode):
    #     np.random.shuffle(node.unexpanded_actions)
    #     action = node.unexpanded_actions.pop()
    #     node_new = self.task_tree.add_child(node, action)
    #     return node_new, action
    
    def make_terminal_node(self, node: TaskNode):
        return self.task_tree.add_child(node, "terminal")
    
    def select_by_uct(self, node: TaskNode):
        if np.random.random() < self.p_terminal:
            return "terminal"
        
        if self.pw_test(node):
            actions = set(node.actions) - set(node.children.keys())
            return choose_random(list(actions))
        actions = list(node.children.keys()) #expanded actions
        np.random.shuffle(actions) #tie breaker
        ucts = []
        for a in actions:
            child = node.children[a]
            n_i = child.visit
            if n_i == 0:
                v_i = 0
            else:
                v_i = child.sum_reward/n_i
            uct = v_i + self.c * np.sqrt(2*np.log(node.visit)/n_i)
            ucts += [uct]
        # if random:
        #     action = choose_random(actions)
        #else:
        i = np.argmax(ucts)
        action = actions[i]
        return action #node.children[action], 
    
    # def select_by_egreedy(self, node: TaskNode):
    #     actions = list(node.children.keys()) #expanded actions
    #     np.random.shuffle(actions) #tie breaker
    #     values = []
    #     for a in actions:
    #         child = node.children[a]
    #         n_i = child.visit
    #         v_i = child.sum_reward/n_i
    #         values += [v_i]
        
    #     if np.random.random() < self.eps:
    #         i = np.argmax(values)
    #         action = actions[i]
    #     else:
    #         action = choose_random(actions)
    #     return node.children[action], action

    def select_by_egreedy2(self, node: TaskNode):
        if np.random.random() < self.p_terminal:
            return "terminal"
        else:
            if np.random.random() < self.eps:
                #exploitation
                actions = list(node.children.keys()) #expanded actions
                np.random.shuffle(actions) #tie breaker
                values = []
                for a in actions:
                    child = node.children[a]
                    n_i = child.visit
                    if n_i == 0: v_i = 0
                    else:v_i = child.sum_reward/n_i
                    values += [v_i]
                i = np.argmax(values)
                return actions[i]
            else:
                return choose_random(node.actions)
        #return node.children[action], action
    
    def update(self, reward:float, pi:List[Operator]):
        node = self.task_tree.root
        for a in pi:
            node.visit += 1
            node.sum_reward += reward
            if a not in node.children: break
            node = node.children[a]

def phi_key(phi:Dict[str,str]):
    values = []
    for obj in np.sort(list(phi.keys())):
        values += [phi[obj]]
    return "_".join(values)

@dataclass
class Node:
    def __init__(
        self, 
        q:Config, 
        theta: Dict[str, Attachment], 
        phi: Dict[str, str],
        traj_switch: List[Config]=None
    ):
        self.q = q
        self.theta = theta
        self.phi = phi
        self.index: int = -1
        self.parent: Node = None
        self.traj_switch = traj_switch
    
    def copy(self):
        q = deepcopy(self.q)
        theta = deepcopy(self.theta)
        phi = deepcopy(self.phi)
        return Node(q, theta, phi)

    @staticmethod
    def get_phi_by_state(state:frozenset)->Dict[str, str]:
        phi = {}
        for pred in state:
            if "attached" not in pred: continue
            obj, parent = split_att_str(pred)
            phi[obj] = parent
        return phi

    @property
    def phi_key(self):
        return phi_key(self.phi)
    
    @property
    def mode(self):
        #trick : in this implementation, phi information is already embedded in theta
        return Mode(self.theta)

    @classmethod
    def from_q_theta_state(cls, q, theta, state):
        phi = cls.get_phi_by_state(state)
        return Node(q, theta, phi)

class GeomGoalSet:
    def __init__(
        self,
        q_goal:Optional[Config] = None,
        att_goal:Optional[Dict[str,str]] = {},
    ):
        self.q_goal = q_goal
        self.att_goal = att_goal

@dataclass
class Trajectory:
    traj:List[Config]
    mode:Mode



class RG:
    """reachability graph
    vertex: (phi, theta, q)
    edge: trajectory
    """
    def __init__(self):
        self.V = []
        self.E = {}
        self.V_table:Dict[Tuple, List[Node]] = {}
        self.sol:Optional[Node] = None
    
    @property
    def root(self)->Node:
        return self.V[0]

    def add_node(self, node:Node):
        index = len(self.V)
        node.index = index
        self.V += [node]
        #self.add_to_table(node, dir)
    
    def add_edge(self, parent:Node, node:Node, traj:Trajectory):
        node.parent = parent
        self.add_traj(parent, node, traj)
    
    def add_traj(self, parent:Node, node:Node, traj:Trajectory):
        self.E[(parent.index, node.index)] = traj
        traj_rev = Trajectory(traj.traj[::-1], traj.mode)
        self.E[(node.index, parent.index)] = traj_rev

    def get_edge(self, node1:Node, node2:Node)->Trajectory:
        return self.E[(node1.index, node2.index)]

    # def add_to_table(self, node:Node, direction:str):
    #     key = direction + ":" + node.phi_key
    #     if key not in self.V_table:
    #         self.V_table[key] = []
    #     self.V_table[key] += [node]
    
    # def sample_node_by_geom_state(self, phi, direction:str):
    #     nodeset = self.nodeset_by_geom_state(phi, direction)
    #     if len(nodeset) != 0:
    #         return choose_random(nodeset)
    #     return None
    
    # def nodeset_by_geom_state(self, phi, direction:str):
    #     key = direction + ":" + phi_key(phi)
    #     if key not in self.V_table:
    #         return []
    #     return self.V_table[key]
    
    


@dataclass
class GeomStateNode:
    phi: Dict[str,str]
    node: Dict[str, List[Node]] = field(default_factory=lambda :{"fwd":[], "bwd":[]})
    param: Dict[str, List[Attachment]] = field(default_factory=lambda :{"fwd":[], "bwd":[]})
    children: Dict[str, "GeomStateNode"] = field(default_factory=lambda :{})
    index: int = field(default_factory=lambda :-1)
    is_end: bool = field(default_factory=lambda :False)

    def add_rg_node_ptr(self, node:Node, dir:str):
        self.node[dir] += [node]
    
    def add_att_param(self, att_param:Attachment, dir:str):
        self.param[dir] += [att_param]

    def sample_rg_node(self, dir:str):
        nodeset = self.node[dir]
        if len(nodeset) == 0:
            return None
        else:
            return choose_random(nodeset)
    
    def sample_att_param(self, dir:str):
        if len(self.param[dir]) == 0:
            return None
        else:
            return choose_random(self.param[dir])
    
    def nodeset(self, dir:str):
        return self.node[dir]
    
    def __repr__(self):
        return f"fwd:{len(self.node['fwd'])},bwd:{len(self.node['bwd'])} phi:{self.phi}"
    

class GeomStateTree:
    def __init__(self):
        self.V = []
    
    @property
    def root(self)->GeomStateNode:
        return self.V[0]
        
    def add_node(self, node:GeomStateNode):
        i = len(self.V)
        node.index = i
        self.V += [node]
    
    def get_geom_state_seq(self, pi:List[Operator]):
        node = self.root
        seq = [node]
        seq[0].is_end = True
        
        for a in pi:
            if a.name in node.children:
                node = node.children[a.name]
            else:
                phi_new = deepcopy(node.phi)
                obj, parent_add, parent_del = a.get_geom_obj_add_del()
                phi_new[obj] = parent_add
                node_new = GeomStateNode(phi_new)
                self.add_node(node_new)
                node.children[a.name] = node_new
                node = node_new
            node.is_end = False
            seq += [node]
        seq[0].is_end = True
        seq[-1].is_end = True
        return seq
        

# class AttachmentTable:
#     def __init__(self):
#         self.table:Dict[Tuple, Attachment] = {}
    
#     def add_attachment(self, att:Tuple, param:Attachment, direction:str):
#         key = (*att, direction)
#         assert att[0] == param.obj_name and att[1] == param.parent_name
#         if key not in self.table:
#             self.table[key] = []
#         self.table[key] += [param]
    
#     def sample_attachment(self, att:Tuple, direction):
#         key = (*att, direction)
#         if key not in self.table:
#             return None
#         else:
#             param = choose_random(self.table[key])
#             return deepcopy(param)

    
class RTH:
    def __init__(
        self, 
        domain_file:str, 
        problem_file:str, 
        domain:TAMPDomain,
        nongeometric_actions:List[str] = [],
        alpha=0.5,
        eps_tp=0.8,
        eps_mmp=0.2,
    ):
        self.sample_table = None
        self.reach_tree_node = []
        self.reach_tree_edge = {}
        self.node_sets = {}
        self.domain = domain
        self.nongeometric_actions = nongeometric_actions
        self.tp = TaskPlanner(domain_file, problem_file, alpha, eps_tp)
        self.eps_mmp = eps_mmp
    
    def plan_fwd_goal_sampling(
        self, q_init:Config, theta_init:Dict[str, Attachment], 
        q_goal:Config=None, theta_goal:Dict[str, Attachment]=None,
    ):
        state_init = self.tp.state_init
        
        root = Node.from_q_theta_state(q_init, theta_init, state_init)
        gs_root = GeomStateNode(root.phi)
        graph = RG()
        graph.add_node(root)
        tree = GeomStateTree()
        gs_root.add_rg_node_ptr(root, "fwd")
        tree.add_node(gs_root)
                
        for i in range(1000):
            pi = self.tp.sample_action_seq()
            pi_g = self.get_geometric_actions(pi)
            for _ in range(2):
                r = self.fixed_skeleton_mmp_gs(graph, tree, pi_g)
                if r is not None:
                    self.tp.update(r, pi)
            if graph.sol is not None:
                self.postprocess_fwd(graph, q_goal)
                break
                
        trajs = self.get_traj(graph)
        self.visualize(trajs)

    def fixed_skeleton_mmp_gs(self, graph:RG, tree:GeomStateTree, pi_g:List[Operator]):
        #goal sampling
        phi_seq = tree.get_geom_state_seq(pi_g)
        atts = self.get_attachments_to_sample(pi_g) 
        theta_goal = deepcopy(graph.root.theta)
        i = 0
        while True:
            params = self.sample_att_params(atts)
            for att in params:
                theta_goal[att.obj_name] = att
            if not self.domain.is_collision(Mode(theta_goal), graph.root.q):
                break
            if i >= 5:
                return None
            i += 1
    
        k = len(atts)
        r_list = []
        for i in range(k):
            phi = phi_seq[i]
            phi_new = phi_seq[i+1]
            att = atts[i]
            param = params[i]
            node = phi.sample_rg_node("fwd")
            if node is None: continue                        
            node_new = self.sample_transition(node, att, param)
            tau = self.get_single_mode_path(node, node_new)
            if tau is not None:
                graph.add_node(node_new)
                graph.add_edge(node, node_new, tau)
                phi_new.add_rg_node_ptr(node_new, "fwd")
                
                if i == (k-1):
                    graph.sol = node_new
            else:
                r_list += [(i+1)/k]
        if len(r_list) != 0: return np.max(r_list)
        else: return None


    def plan_fwd(
        self, q_init:Config, theta_init:Dict[str, Attachment], 
        q_goal:Config=None, theta_goal:Dict[str, Attachment]=None,
    ):
        state_init = self.tp.state_init
        graph = RG()
        root = Node.from_q_theta_state(q_init, theta_init, state_init)
        graph.add_node(root)
        tree = GeomStateTree()
        gs_root = GeomStateNode(root.phi)
        gs_root.add_rg_node_ptr(root, "fwd")
        tree.add_node(gs_root)
                
        for i in range(1000):
            pi = self.tp.sample_action_seq()
            pi_g = self.get_geometric_actions(pi)
            #for i in range(5):
            r = self.fixed_skeleton_mmp(graph, tree, pi_g)
            if r is not None:
                self.tp.update(r, pi)
            if graph.sol is not None:
                self.postprocess_fwd(graph, q_goal)
                break
                
        trajs = self.get_traj(graph)
        self.visualize(trajs)
    
    def plan_bwd(
        self, q_init:Config, theta_init:Dict[str, Attachment], 
        q_goal:Config=None, theta_goal:Dict[str, Attachment]=None,
    ):
        state_init = self.tp.state_init
        graph = RG()
        root = Node.from_q_theta_state(q_init, theta_init, state_init)
        graph.add_node(root)
        tree = GeomStateTree()
        gs_root = GeomStateNode(root.phi)
        gs_root.add_rg_node_ptr(root, "fwd")
        tree.add_node(gs_root)
        
        goalset = GeomGoalSet(q_goal = q_goal)
        for i in range(1000):
            pi = self.tp.sample_action_seq()
            pi_g = self.get_geometric_actions(pi)
            #for i in range(5):
            r = self.fixed_skeleton_bwdmmp(graph, tree, pi_g, goalset)
            
            if r is not None:
                self.tp.update(r, pi)
            
            if graph.sol is not None:
                self.postprocess_fwd(graph, q_goal)
                break
                
        trajs = self.get_traj(graph)
        self.visualize(trajs)

    def plan_bi(
        self, q_init:Config, theta_init:Dict[str, Attachment], 
        q_goal:Config=None, theta_goal:Dict[str, Attachment]=None,
        c=1, eps_task=0.5, eps_reach=0.2, alpha=0.5
    ):
        self.alpha = alpha # pw parameter
        state_init = self.tp.state_init
        graph = RG()
        root = Node.from_q_theta_state(q_init, theta_init, state_init)
        graph.add_node(root)
        tree = GeomStateTree()
        gs_root = GeomStateNode(root.phi)
        gs_root.add_rg_node_ptr(root, "fwd")
        tree.add_node(gs_root)

        goalset = GeomGoalSet(q_goal = q_goal)
        start = time.perf_counter()
        for i in range(1000):
            pi = self.tp.sample_action_seq()
            pi_g = self.get_geometric_actions(pi)
            for _ in range(2):
                r = self.fixed_skeleton_bimmp(graph, tree, pi_g, goalset)
            
            if graph.sol is not None:
                #self.postprocess_fwd(graph, q_goal)
                break
            self.tp.update(r, pi)
        end = time.perf_counter()
        print(f"elapsed: {end-start}")
        self.visualize_bi(graph)

    def fixed_skeleton_bwdmmp(
        self, graph:RG, tree:GeomStateTree,
        pi_f:List[Operator], goalset:GeomGoalSet
    ):
        phi_list = tree.get_geom_state_seq(pi_f)
        i = 0
        while True:
            if i > 10: return None
            atts = self.get_attachments_to_sample(pi_f)
            att_params = self.sample_att_params(atts)
            goal_theta = self.check_extend_plan(graph.root, att_params)
            if goal_theta is not None:
                break
            i += 1
        goal_node = Node(goalset.q_goal, goal_theta, phi_list[-1].phi)
        graph.add_node(goal_node)
        phi_list[-1].add_rg_node_ptr(goal_node, "bwd")

        pi_b = self.reverse_plan(pi_f)
        atts_b = self.get_attachments_to_sample(pi_b)
        att_params_b = self.sample_att_params(atts_b)
        #atts = self.get_attachments_to_sample(pi_f)
        return self.batch_extend_bwd(graph, phi_list[::-1], atts_b, att_params_b)

    def batch_extend_bwd(
        self, 
        graph:RG, 
        phi_list:List[GeomStateNode],
        atts:List[Tuple], 
        att_params:List[Attachment],
        #direction:str
    ):  
        k = len(atts)
        r_list = []
        for i in range(k):
            phi = phi_list[i]
            phi_new = phi_list[i+1]
            att = atts[i]

            #att = atts[i]
            node = phi.sample_rg_node("bwd")
            if node is None: continue
            att_param = self.sample_param(node, att)
            # if att_param is None:
            #     att_param = att_params[i]    
            
            node_new = self.sample_transition(node, att, att_param)
            tau = self.get_single_mode_path(node, node_new)
            if tau is not None:
                graph.add_node(node_new)
                graph.add_edge(node, node_new, tau)
                phi_new.add_rg_node_ptr(node_new, "bwd")
                self.memorize_sample(node, att, att_param, True)
                if i == (k-1):
                    print('goal')
            else:
                r_list += [(i+1)/k]
                self.memorize_sample(node, att, att_param, False)
        return np.max(r_list)

    def fixed_skeleton_bimmp(
        self, graph:RG, tree:GeomStateTree,
        pi_f:List[Operator], goalset:GeomGoalSet
    ):
        #init
        node_start = graph.root
        phi_seq = tree.get_geom_state_seq(pi_f)
        pi_b = self.reverse_plan(pi_f)
        node_goal = self.sample_goal_node(graph.root, pi_f, goalset)
        if node_goal is None: return 0
        graph.add_node(node_goal)
        phi_seq[-1].add_rg_node_ptr(node_goal, "bwd")
        for dir, node in zip(["bwd", "fwd"], [node_start, node_goal]):
            for att in node.phi.items():
                param = node.theta[att[0]]
                phi = self.find_phi_to_add_param(phi_seq, att)
                if phi is not None:
                    phi.add_att_param(param, dir)
            phi_seq = phi_seq[::-1]
        
        
        #bwd
        pi = {dir:obj for obj, dir in zip([pi_f, pi_b],["fwd", "bwd"])}
        for dir in ["bwd", "fwd"]:
            phi_seq = phi_seq[::-1]
            atts = self.get_attachments_to_sample(pi[dir]) 
            att_params = self.sample_att_params2(atts, phi_seq, dir)
            successes = self.batch_extend(graph, phi_seq, atts, att_params, dir)        
            if graph.sol is not None:
                return 1
            self.register_att_params(phi_seq, successes, atts, att_params, dir)
            
        r = self.get_reward(successes)
        
        return r
    
    
    def batch_extend(
        self, graph:RG, phi_list:List[GeomStateNode], 
        atts:List[Tuple], att_params:List[Attachment],
        direction:str
    ):
        k = len(atts)
        success = []
        for i in range(k):
            phi = phi_list[i]
            phi_new = phi_list[i+1]
            att, att_param = atts[i], att_params[i]
            node = phi.sample_rg_node(direction)
            node_new = self.sample_transition(node, att, att_param)
            tau = self.get_single_mode_path(node, node_new)
            if tau is not None:
                graph.add_node(node_new)
                graph.add_edge(node, node_new, tau)
                phi_new.add_rg_node_ptr(node_new, direction)
                success += [True]
            
            node_last = self.sol_candidate(node_new, phi_new, direction)
            if node_last is not None:
                tau = self.get_single_mode_path(node_new, node_last)
                if tau is not None:
                    graph.add_traj(node_new, node_last, tau)
                    self.postprocess(graph, node_new, node_last, direction)
                    return None
            else:
                success += [False]
        return success
    
    def visualize_bi(self, graph:RG):
        node_seq = graph.sol
        trajs:List[Trajectory] = []
        for i in range(len(node_seq)-1):
            node1:Node = node_seq[i]
            node2:Node = node_seq[i+1]
            traj = graph.get_edge(node1, node2)
            if node1.traj_switch is not None:
                traj.traj = node1.traj_switch[::-1] + traj.traj
            if node2.traj_switch is not None:
                traj.traj += node2.traj_switch
            trajs += [traj]
        
        for traj in trajs:
            mode = traj.mode
            for config in traj.traj:
                self.domain.assign(mode, config)
                time.sleep(0.05)


    def postprocess(self, graph:RG, node_new, node_last, direction):
        if direction == "fwd":
            node1, node2 = node_new, node_last
        else:
            node2, node1 = node_new, node_last
        node_seq = []
        node_seq += self.backtrack(node1)
        node_seq += self.backtrack(node2)[::-1]
        graph.sol = node_seq
    
    def backtrack(self, node:Node):
        seq = []
        while node is not None:
            seq += [node]
            node = node.parent
        return seq[::-1]

    def sol_candidate(self, node_new:Node, phi_new:GeomStateNode, direction:str):
        if node_new is None: return None
        rev_dir = "bwd" if direction == "fwd" else "fwd"
        nodeset = phi_new.nodeset(rev_dir)
        
        # if fwd search successes
        if phi_new.is_end == True:
            node_end = nodeset[0]
            node_end.theta = deepcopy(node_new.theta)
            return node_end
        
        #if search meets in the middle
        for node in nodeset:
            if node.mode == node_new.mode:
                return node
        return None

    def register_att_params(self, phi_seq:List[GeomStateNode], successes, atts, att_params, direction):
        rev_dir = "bwd" if direction == "fwd" else "fwd"
        k = len(atts)
        for i in range(k-1):
            if not successes[i]: continue
            att, att_param = atts[i], att_params[i]
            phi = self.find_phi_to_add_param(phi_seq[i+1:], att)
            if phi is not None:
                phi.add_att_param(att_param, rev_dir)
            
    def find_phi_to_add_param(
        self, phi_seq:List[GeomStateNode], att
    )->GeomStateNode:
        for i, phi_node in enumerate(phi_seq[:-1]):
            phi_curr = phi_node.phi
            phi_next = phi_seq[i+1].phi
            for obj in phi_curr:
                if phi_curr[obj] != phi_next[obj] and att == (obj, phi_curr[obj]):
                    return phi_node
        return None

    def sample_att_params2(self, atts, phi_seq:List[GeomStateNode], direction:str):
        params = []
        for i, att in enumerate(atts):
            phi_next = phi_seq[i+1]
            param = None
            obj, parent = att
            if direction == "bwd" or np.random.random() < self.eps_mmp:
                param = phi_next.sample_att_param(direction)
            if param is None:
                param = self.domain.sample_attachment(obj, parent)
            params += [param]
        return params
    
    def reverse_plan(self, pi:List[Operator]):
        return [a.reverse() for a in pi][::-1]

    def sample_goal_node(self, node:Node, pi:List[Operator], goalset:GeomGoalSet, max_iter=10):
        phi = deepcopy(node.phi)
        unchanged_atts = deepcopy(node.phi)
        for a in pi:
            obj, parent_add, parent_del = a.get_geom_obj_add_del()
            if obj in unchanged_atts:
                unchanged_atts.pop(obj)
            phi[obj] = parent_add
        changed_atts = {k:v for k,v in phi.items() if k not in unchanged_atts}

        node_goal = node.copy()
        node_goal.phi = phi
        for _ in range(max_iter):
            for obj, parent in changed_atts.items():
                if obj in goalset.att_goal:
                    node_goal.theta[obj] = goalset.att_goal[obj]
                else:
                    node_goal.theta[obj] = self.domain.sample_attachment(obj, parent)
            if goalset.q_goal is not None:
                node_goal.q = goalset.q_goal
            else:
                robot:Panda = self.domain.robots["robot"]
                node_goal.q.set_q("robot", robot.get_random_joint_angles())
            
            if not self.domain.is_collision(Mode(node_goal.theta), node_goal.q):
                return node_goal
        return None
                
    def get_traj(self, graph:RG):
        trajs =[]
        node:Node = graph.sol
        traj_switch = None
        while True:
            parent = node.parent
            if parent is None: break
            traj = graph.get_edge(parent, node)
            if parent.traj_switch is not None:
                traj.traj = parent.traj_switch[::-1] + traj.traj
            if node.traj_switch is not None:
                traj.traj += node.traj_switch
            trajs += [traj]
            node = parent
        return trajs[::-1]

    def visualize(self, trajs:List[Trajectory]):
        for traj in trajs:
            mode = traj.mode
            for config in traj.traj:
                self.domain.assign(mode, config)
                time.sleep(0.01)

    def get_geometric_actions(self, pi:List[Operator]):
        pi_g = [a for a in pi if a.is_geom_action()]
        return pi_g
    
    def fixed_skeleton_mmp(self, graph:RG, tree:GeomStateTree, pi:List[Operator]):
        phi_list = tree.get_geom_state_seq(pi)
        atts = self.get_attachments_to_sample(pi)
        # i = 0
        # while True:
        #     if i > 10: return None
        #     atts = self.get_attachments_to_sample(pi)
        #     att_params = self.sample_att_params(atts)
        #     if self.check_extend_plan(graph.root, att_params):
        #         break
        #     i += 1
        
        return self.batch_extend_fwd(graph, phi_list, atts)
    
    def batch_extend_fwd(
        self, 
        graph:RG, 
        phi_list:List[GeomStateNode],
        atts:List[Tuple], 
        #att_params:List[Attachment],
        #direction:str
    ):
        k = len(atts)
        r_list = []
        for i in range(k):
            phi = phi_list[i]
            phi_new = phi_list[i+1]
            att = atts[i]

            #att = atts[i]
            node = phi.sample_rg_node("fwd")
            if node is None: continue
            att_param = self.sample_param(node, att)
            # if att_param is None:
            #     att_param = att_params[i]    
            
            node_new = self.sample_transition(node, att, att_param)
            tau = self.get_single_mode_path(node, node_new)
            if tau is not None:
                graph.add_node(node_new)
                graph.add_edge(node, node_new, tau)
                phi_new.add_rg_node_ptr(node_new, "fwd")
                #self.memorize_sample(node, att, att_param, True)
                if i == (k-1):
                    print('goal')
            else:
                r_list += [(i+1)/k]
                #self.memorize_sample(node, att, att_param, False)
        return np.max(r_list)

    def make_node_att_key(self, node:Node, att:Attachment):
        obj, parent_add = att
        att_key = "_".join(att)
        if self.sample_table is None:
            self.sample_table = {}
        mode_key = node.mode.get_hashable_key()
        return "_".join([att_key, mode_key])

    def sample_param(self, node:Node, att:Tuple):
        obj, parent_add = att
        key = self.make_node_att_key(node, att)
        self.domain.assign(node.mode)
        rand_sample = False
        if key not in self.sample_table: rand_sample = True
        #elif len(self.sample_table[key]["succ"]) == 0: rand_sample = True
        elif len(self.sample_table[key]) == 0: rand_sample = True
        #elif np.random.random() > 0.5: random_sample = True
        
        if rand_sample:
            return self.domain.sample_attachment(obj, parent_add) #None #
        else:
            while True:
                param = self.domain.sample_attachment(obj, parent_add)
                min_d = np.inf
                result = None
                for att, res in self.sample_table[key]:
                    d = self.domain.heuristic_distance(param, att)
                    print(d)

                    if min_d >= d:
                        min_d = d
                        result = res
                if result == "succ":
                    return param
                elif d >= 0.5 or np.random.random() < 0.2:
                    return param
                else: 
                    print("reject")
                    continue
                

                
                # succ_samples = self.sample_table[key]["succ"]
                # fail_samples = self.sample_table[key]["fail"]
                # min_dist_succ = np.min([self.att_distance(att, param) for att in succ_samples])
                # min_dist_fail = np.min([self.att_distance(att, param) for att in fail_samples])
                # if min_dist_succ < min_dist_fail or abs(min_dist_succ-min_dist_fail) < 0.01:
                #     return param
    
    def memorize_sample(self, node, att, param, is_success):
        obj, parent_add = att
        key = self.make_node_att_key(node, att)
        if key not in self.sample_table:
            self.sample_table[key] = []
        if is_success:
            self.sample_table[key] += [(param, "succ")]
        else:
            self.sample_table[key] += [(param, "fail")]
    
    def att_distance(self, att1:Attachment, att2:Attachment):
        if isinstance(att1, Grasp) and isinstance(att2, Grasp):
            tf1 = att1.tf * Pose(trans=[0,0,-att1.pre_pose_distance])
            tf2 = att2.tf * Pose(trans=[0,0,-att2.pre_pose_distance])
            return np.linalg.norm(tf1.trans - tf2.trans)
        elif isinstance(att1, Placement) and isinstance(att2, Placement):
            return np.linalg.norm(att1.tf.inverse().trans - att2.tf.inverse().trans)
    
    def check_extend_plan(self, root:Node, att_params:List[Attachment]):
        theta = deepcopy(root.theta)
        for att in att_params:
            theta[att.obj_name] = att
        if not self.domain.is_collision(Mode(theta), root.q):
            return theta
        return None

    def get_reward(self, successes):
        r = 0
        k = len(successes)
        for i, success in enumerate(successes[::-1]):
            if success:
                break
        r = (k-i)/k
        return r

    def sample_att_param(self, att:Tuple):
        obj, parent = att
        param = self.domain.sample_attachment(obj, parent)
        return param

    def postprocess_fwd(self, graph:RG, q_goal:Config):
        node_last = graph.sol.copy()
        node_last.q = q_goal
        tau = self.get_single_mode_path(graph.sol, node_last)
        graph.add_node(node_last)
        graph.add_edge(graph.sol, node_last, tau)
        graph.sol = node_last

    def get_single_mode_path(self, node:Node, node_new:Node):
        if node_new is None: return None
        mode = node.mode
        mp = BiRRT2(q_delta_max=0.1, ts_eps=0.01)
        traj = mp.plan(
            ["robot"], node.q, node_new.q, mode, self.domain)
        if traj is not None: 
            return Trajectory(traj, mode)
        return None

    def sample_transition(self, node:Node, att:str, att_param:Attachment):
        if node is None: return None
        mode = node.mode
        self.domain.assign(mode)

        obj, parent = att
        if "robot" in parent: #pick action
            grasp = att_param
            placement = mode.attachments[obj]
            placement_parent = mode.attachments[obj].parent_name
        else: #place action
            grasp = mode.attachments[obj]
            placement = att_param
            placement_parent = parent
        
        parent_pose = self.domain.objects[placement_parent].get_base_pose()
        obj_pose = parent_pose * placement.tf.inverse()
        grasp_pose = obj_pose * grasp.tf
        grasp_pose_pre = grasp.get_pre_pose(grasp_pose)
        
        robot = self.domain.robots["robot"]
        q_pre_ik = robot.inverse_kinematics(pose=grasp_pose_pre)
        if q_pre_ik is None: return None
        
        theta_new = deepcopy(node.theta)
        config_pre = deepcopy(node.q)
        phi_new = deepcopy(node.phi)
        theta_new[obj] = att_param
        phi_new[obj] = parent
        config_pre.set_q("robot", q_pre_ik)
        
        mode_new = Mode(theta_new)
        if self.domain.is_collision(mode_new, config_pre): 
            return None
        
        mp = BiRRT2(ts_eps=0.02)
        traj_switch = mp.check_mode_switch(
            ["robot"], config_pre, grasp_pose, mode, self.domain)
        if traj_switch is not None: 
            return Node(config_pre, theta_new, phi_new, traj_switch=traj_switch)
        return None

    def get_attachments_to_sample(self, pi:List[Operator]):
        atts = []
        for a in pi:
            atts += [self.get_next_att(a)]
        return atts
    
    def get_next_att(self, a:Operator):
        obj, parent_add, parent_del = a.get_geom_obj_add_del()
        return (obj, parent_add)

    def sample_att_params(self, atts)->List[Attachment]:
        params = []
        for att in atts:
            (obj, parent) = att
            param = self.domain.sample_attachment(obj, parent)
            params += [param]
        return params
                
    def apply_geom_action(self, phi:Dict[str, str], a:Operator):
        obj, parent_add, parent_del = a.get_geom_obj_add_del()
        phi = deepcopy(phi)
        phi[obj] = parent_add
        return phi

    def get_geom_state_seq(self, phi:Dict[str,str], pi:List[Operator]):
        phi = deepcopy(phi)
        seq = [phi]
        for a in pi:
            obj, parent_add, parent_del = a.get_geom_obj_add_del()
            phi = deepcopy(phi)
            phi[obj] = parent_add
            seq += [phi]
        return seq

        

if __name__ == "__main__":
    domain_name = "packing"

    if domain_name == "kitchen3":
        dom = DomainKitchen(gui=True, num_box=3)
        prob = ProblemKitchen(dom, num_block=3, cooked_list=[1, 2, 3])
        domain_file = Path(__file__).parent.parent / "domain/kitchen/pddl/domain.pddl"
        problem_file = Path(__file__).parent.parent / "domain/kitchen/pddl/problem_box3.pddl"
        nongeometric_actions = ["wash", "cook"]
    if domain_name == "kitchen4":
        dom = DomainKitchen(gui=True, num_box=4)
        prob = ProblemKitchen(dom, num_block=4, cooked_list=[1, 2, 3, 4])
        domain_file = Path(__file__).parent.parent / "domain/kitchen/pddl/domain.pddl"
        problem_file = Path(__file__).parent.parent / "domain/kitchen/pddl/problem_box4.pddl"
        nongeometric_actions = ["wash", "cook"]
    if domain_name == "kitchen5":
        dom = DomainKitchen(gui=True, num_box=5)
        prob = ProblemKitchen(dom, num_block=5, cooked_list=[1, 2, 3, 4, 5])
        domain_file = Path(__file__).parent.parent / "domain/kitchen/pddl/domain.pddl"
        problem_file = Path(__file__).parent.parent / "domain/kitchen/pddl/problem_box5.pddl"
        nongeometric_actions = ["wash", "cook"]
    elif domain_name == "hanoi":
        dom = DomainHanoi(gui=True)
        prob = ProblemHanoi(dom)
        domain_file = Path(__file__).parent.parent / "domain/hanoi/pddl/domain.pddl"
        problem_file = Path(__file__).parent.parent / "domain/hanoi/pddl/problem_ex.pddl"
    elif domain_name == "unpacking":
        dom = DomainUnpacking(gui=True)
        prob = ProblemUnpacking(dom)
        domain_file = Path(__file__).parent.parent / "domain/unpacking/pddl/domain.pddl"
        problem_file = Path(__file__).parent.parent / "domain/unpacking/pddl/problem.pddl"
    elif domain_name == "packing":
        dom = DomainPacking(gui=True)
        prob = ProblemPacking(dom)
        domain_file = Path(__file__).parent.parent / "domain/packing/pddl/domain.pddl"
        problem_file = Path(__file__).parent.parent / "domain/packing/pddl/problem.pddl"
    
    # elif domain_name == "Packing":
    #     dom = DomainPacking(gui=True)
    #     prob = ProblemPacking(dom)
    #     domain_file = Path(__file__).parent.parent / "domain/packing/pddl/domain.pddl"
    #     problem_file = Path(__file__).parent.parent / "domain/packing/pddl/problem.pddl"

    rth = RTH(
        domain_file, problem_file, dom, 
    )
    q = prob.config_init
    theta = deepcopy(prob.mode_init.attachments)

    rth.plan_fwd_goal_sampling(
        q, theta, q_goal=q
    )
