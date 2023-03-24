from numpy import choose
from sympy import Q
from bmp import *
from pyperplan.task import Operator, Task
from pyperplan.planner import HEURISTICS, SEARCHES, _parse, _ground, _search
from pyperplan.pddl.parser import Parser
from typing import TypeVar
from dataclasses import dataclass

T = TypeVar('T')

def choose_random(x:List[T])->T:
    i = np.random.randint(0, len(x))
    return x[i]

def phi(state:frozenset):
    phi = [pred for pred in state if "attached" in pred]
    phi_dict = {}
    for _phi in phi:
        _, obj, parent = _phi.strip("()").split(" ")
        phi_dict[obj] = parent
    return phi_dict

    
class NodeTable:
    def __init__(self):
        self.table = {}
    
    def make_key(self, state:frozenset, dir:str):
        state_key = "".join(np.sort(list(state)))
        return dir+state_key

    def nodeset(self, state:frozenset, dir:str)-> List["Node"]:
        key = self.make_key(state, dir)
        return self.table[key]
    
    def add(self, state:frozenset, dir:str, node:"Node"):
        key = self.make_key(state, dir)
        if key not in self.table:
            self.table[key] = []
        self.table[key] += [node]

class GuideTable:
    def __init__(self):
        self.table = {}
    
    def make_key(self, state:frozenset, action:Operator, dir:str):
        state_key = "".join(np.sort(list(state)))
        action_key = action.name
        return dir + state_key + action_key

    def guideset(self, state:frozenset, action:Operator, dir:str):
        key = self.make_key(state, action, dir)
        if key not in self.table:
            return []
        return self.table[key]
    
    def add(self, state:frozenset, action:Operator, dir:str, guide):
        key = self.make_key(state, action, dir)
        if key not in self.table:
            self.table[key] = []
        self.table[key] += [guide]

@dataclass
class TaskNode:
    state: frozenset
    unexpanded_actions: List[Operator]
    parent: Optional["TaskNode"] = field(default_factory=lambda :None)
    children: Dict[Operator,"TaskNode"] = field(default_factory=lambda :{})
    visit: int = field(default_factory=lambda :0)
    sum_reward: int = field(default_factory=lambda :0)
    is_terminal: bool = field(default_factory=lambda : False)
    index: int = field(default_factory=lambda :-1)
    
    @property
    def fully_expanded(self):
        return len(self.unexpanded_actions) == 0

@dataclass
class TaskNodeTerminal(TaskNode):
    state: Optional[frozenset] = field(default_factory=lambda :None)
    unexpanded_actions: Optional[List] = field(default_factory=lambda :[])
    is_terminal: bool = field(default_factory=lambda :True)

class Node:
    def __init__(
        self, 
        q: Config, 
        theta: Dict[str, Attachment] = None, 
        state: Optional[frozenset] = None,
        parent: Optional["Node"] = None,
        traj_switch: Optional[List[Config]] = None
    ):
        self.q = q
        self.theta = theta
        self.state = state
        self.parent = parent
        self.visit = 0
        self.sum_reward = 0    
        self.index: int = -1
        self.traj_switch = traj_switch
    
    @property
    def phi(self):
        result = []
        for pred in self.state:
            name, *_ = pred.strip("()").split(" ")
            if name == "attached":
                result += [pred]
        return result

    @property
    def value(self):
        if self.visit == 0: return 0
        return self.sum_reward/self.visit

    def __le__(self, other):
        return self.value <= other.value

class GoalSet(Node):
    def __init__(
        self, 
        q: Config = None, 
        theta: Dict[str, Attachment] = None, 
        state: Optional[frozenset] = None,
    ):
        super().__init__(q=q, theta=theta, state=state)


class TaskTree:
    def __init__(self, domain_file:str, problem_file:str):
        self.domain_file = domain_file 
        self.problem_file = problem_file
        
        self.parser = Parser(
            self.domain_file, probFile=self.problem_file)
        self.domain = self.parser.parse_domain()
        self.problem = self.parser.parse_problem(self.domain)
        self.task = _ground(self.problem)

        self.search = SEARCHES["gbf"]
        heuristic_class = HEURISTICS["hadd"]
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

class TMTS:
    def __init__(
        self, 
        domain_file:str, 
        problem_file:str, 
        domain:TAMPDomain,
        nongeometric_actions:List[str] = []
    ):
        self.task_tree = TaskTree(domain_file, problem_file)
        self.reach_tree_node = []
        self.reach_tree_edge = {}
        self.node_sets = {}
        self.domain = domain
        self.nongeometric_actions = nongeometric_actions

    def sample_goal(
        self, v_init:Node, pi:List[Operator], X_goal:GoalSet, max_iter=10
    )->Node:
        state = v_init.state
        unchanged_preds = deepcopy(v_init.state)
        for a in pi:
            state = a.apply(state)
            unchanged_preds = unchanged_preds.difference(a.del_effects)
        changed_preds = state.difference(unchanged_preds)
        
        v_goal = deepcopy(v_init)
        v_goal.state = state
        for _ in range(max_iter):
            for pred in changed_preds:
                pred_name, *other = pred.strip("()").split(" ")
                if pred_name != "attached": continue
                obj, parent = other
                if obj in self.theta_goal: 
                    v_goal.theta[obj] = deepcopy(self.theta_goal[obj])
                # sample
                if "robot" in parent:
                    v_goal.theta[obj] = self.domain.sample_grasp(obj, parent)
                else:
                    v_goal.theta[obj] = self.domain.sample_placement(obj, parent)
            if not self.domain.is_collision(Mode(v_goal.theta), v_goal.q):
                return v_goal
        return None

    def plan_bi(
        self, q:Config, theta:Dict[str, Attachment], 
        q_goal:Config=None, 
        theta_goal:Dict[str, Attachment]=None,
        c=1, eps_task=0.2, eps_reach=0.2, alpha=0.5
    ):
        self.alpha = alpha
        state_init = self.task_tree.state_init
        v_init = Node(q, theta, state=state_init)
        X_goal = GoalSet(q_goal, theta_goal)
        self.add_mode_node(v_init, v_init.state)
        Vset, Gset = NodeTable(), GuideTable()
        Vset.add(state_init, "fwd", v_init)

        for _ in range(1000):
            node, pi_tree = self.task_tree_policy(eps=eps_task)
            pi_plan = self.get_task_plan(node.state)
            pi = pi_tree + pi_plan
            reward = self.reachability_tree_search_bi(
                v_init, pi, Vset, Gset, X_goal, eps=eps_reach)
            if reward == 1:
                return True
            else:
                self.backpropagate(node, reward)
    
    def reachability_tree_search_bi(
        self, 
        v_init:Node,
        pi_fwd:List[Operator],
        V:NodeTable, #Dict[frozenset, List],
        G:GuideTable, #Dict[frozenset, List],
        X_goal:GoalSet,
        eps = 0.5
    ):
        #sample goal
        v_goal = self.sample_goal(v_init, pi_fwd, X_goal)
        if v_goal is None: return 0.
        self.add_mode_node(v_goal, v_goal.state)
        pi_bwd = [self.action_reverse(a) for a in pi_fwd[::-1]]
        V.add(v_init.state, "bwd", v_goal)
        
        self.propagate_state(v_init, pi_fwd, "bwd", G)
        self.propagate_state(v_goal, pi_bwd, "fwd", G)

        r_max = 0
        dir, rev = "fwd", "bwd"
        k = len(pi) - 1
        for _ in range(2):
            pi = pi_fwd if dir == "fwd" else pi_bwd    
            state = v_init.state if dir == "fwd" else v_goal.state

            for i, a in enumerate(pi):
                if len(V.nodeset(state, dir)) == 0: break
                v_parent = choose_random(V.nodeset(state, dir))
                
                state_new = a.apply(v_parent.state)
                if self.is_nongeometric_action(a):
                    v_new = Node(deepcopy(v_parent.q), deepcopy(v_parent.theta))
                    tau = []
                    self.add_mode_node(v_new, state_new, v_parent)
                    self.add_edge(v_parent, v_new, tau)
                else:
                    if np.random.random() < eps and len(G.guideset(state, a, dir)) != 0:
                        theta_add = choose_random(G.guideset(state, a, dir))
                    else:
                        theta_add = self.sample_theta_add(v_parent, a)
                    v_new = self.extend(v_parent, a, theta_add)
                    if v_new is not None:
                        V.add(state_new, dir, v_new)
                        name, obj, phi_del, phi_add = a.name.strip("()").split(" ")
                        self.add_guide(phi_add, theta_add, state_new, pi[i+1:], G)
                        
                        #goal condition
                        if self.is_goal_condition(v_new, V):
                            v_rev = self.get_final_node()
                            tau = self.get_single_mode_path(phi(state), v_new, v_rev)
                            if tau is not None:
                                self.postprocess()
                
                q_goal = None
                v_rev = search_same_mode_in_opposite_dir(dir, v_new, V)
                if v_rev is not None:
                    q_goal = v_rev.q
                    tau = self.get_single_mode_path(phi(state), v_new, v_rev)
                elif dir == "fwd" and self.is_in_goalset():
                    q_goal = v_goal.q
                elif self.two_tree_meets(dir, v_new, V):
                    pass
                elif self.two_tree_meets(dir, v_new, V):
                    same_mode_nodes = [v for v in V.nodeset(state_new, rev) if v.theta == v_new.theta]
                    v_rev = choose_random(same_mode_nodes)

                    
                    if tau is not None:
                        self.add_edge(v_new, v_rev, tau)
                        self.solution = (dir, v_new, v_rev)
                        return 1.
                else:
                    r_max = np.max([r_max, (i+1)/(len(pi))]) 
                    state = state_new    
                    
        return r_max

    # def plan_fwd(
    #     self, q:Config, theta:Dict[str, Attachment], 
    #     eps_task=0.2, alpha=0.5
    # ):
    #     self.alpha = alpha
    #     v_init = Node(q, theta)
    #     state_init = self.task_tree.state_init
    #     h_fwd = {}
    #     nodeset_init = self.get_nodeset_by_state(h_fwd, state_init)
    #     nodeset_init += [v_init]

    #     for _ in range(100):
    #         node, pi_tree = self.task_tree_policy(eps=eps_task)
    #         pi_plan = self.get_task_plan(node.state)
    #         pi = pi_tree + pi_plan
    #         state_list = self.get_state_sequence(state_init, pi)
    #         reward = self.reachability_tree_search(state_list, pi, h_fwd)
    #         if reward == 1:
    #             return True
    #         else:
    #             self.backpropagate(node, reward)
    
    def plan_bi(
        self, q:Config, theta:Dict[str, Attachment], 
        q_goal:Config=None, 
        theta_goal:Dict[str, Attachment]=None,
        c=1, eps_task=0.2, eps_reach=0.2, alpha=0.5
    ):
        self.alpha = alpha
        state_init = self.task_tree.state_init
        v_init = Node(q, theta, state=state_init)
        X_goal = GoalSet(q_goal, theta_goal)
        self.add_mode_node(v_init, v_init.state)
        Vset, Gset = NodeTable(), GuideTable()
        Vset.add(state_init, "fwd", v_init)

        for _ in range(1000):
            node, pi_tree = self.task_tree_policy(eps=eps_task)
            pi_plan = self.get_task_plan(node.state)
            pi = pi_tree + pi_plan
            reward = self.reachability_tree_search_bi(
                v_init, pi, Vset, Gset, X_goal, eps=eps_reach)
            if reward == 1:
                return True
            else:
                self.backpropagate(node, reward)
    
    # TASK Tree Search
    def task_tree_policy(self, eps):
        pi_tree = []
        node = self.task_tree.root
        # selection
        while True:
            node.visit += 1
            if node.is_terminal:
                break
            elif (len(node.children) == 0) or (self.pw_test(node)):
                #print("new_task_plan")
                node_new, action = self.expand_new_child(node)
                node_new.visit += 1
                pi_tree += [action]
                node = self.make_terminal_node(node_new)
            else:
                #node, action = self.select_by_uct(node, c=c, random=random)
                node, action = self.select_by_egreedy(node, eps=eps)
                if not node.is_terminal: pi_tree += [action]
        node_last = node.parent
        return (node_last, pi_tree)
    
    def get_state_sequence(self, state:frozenset, pi:List[Operator]):
        seq = [state]
        for a in pi:
            state = a.apply(state)
            seq += [state]
        return seq
    
    def is_nongeometric_action(self, a:Operator):
        name, *_ = a.name.strip("()").split(" ")
        for action_name in self.nongeometric_actions:
            if action_name in name:
                return True
        return False

    def reachability_tree_search(
        self, 
        states:List[frozenset], 
        pi:List[Operator],
        h_fwd:Dict[frozenset, List]
    ):
        for i, a in enumerate(pi):
            state_prev, state_curr = states[i], states[i+1]
            nodeset_prev = self.get_nodeset_by_state(h_fwd, state_prev)
            nodeset_curr = self.get_nodeset_by_state(h_fwd, state_curr)
            if len(nodeset_prev) == 0: break

            v_prev = choose_random(nodeset_prev)
            
            if self.is_nongeometric_action(a): 
                v_new = Node(deepcopy(v_prev.q), deepcopy(v_prev.theta))
            else:
                theta_new = self.sample_mode_param(v_prev, a)
                v_new = self.sample_transition(v_prev, a, theta_new) #trick: we check ik of pre motion in it
                tau = self.get_single_mode_path(phi(states[i]), v_prev, v_new)
                r = i/(len(pi)-1)
                if tau is None: continue
            
            self.add_mode_node(v_prev, v_new, state_curr, tau)
            nodeset_curr += [v_new]
        
        if len(self.get_nodeset_by_state(h_fwd, states[-1])) != 0: 
            return 1.
        else: return r

    def add_guide(self, dir, phi_add, theta_add, state, pi:List[Operator], guide_table:GuideTable):
        for a in pi:
            state = a.apply(state)
            a_rev = self.action_reverse(a)
            if phi_add in a_rev.add_effects:
                guide_table.add(state, a_rev, dir, theta_add)
                break

    def propagate_state(self, v:Node, pi:List[Operator], dir, Gset):
        # set guides
        for phi_add in [pred for pred in v.state if self.is_phi(pred)]:
            name, obj, parent = phi_add.strip("()").split(" ")
            theta_add = v.theta[obj]
            self.add_guide(dir, phi_add, theta_add, v.state, pi, Gset)

    def is_phi(self, pred):
        name, *_ = pred.strip("()").split(" ")
        if name == "attached":
            return True
        return False

    

    def extend(self, v_parent:Node, a:Operator, theta_add:Attachment):
        raise NotImplementedError()

    def sample_theta_add(self, v:Node, a:Operator) -> Attachment:
        raise NotImplementedError()

    def postprocess(self):
        (direction, v_new, v_rev) = self.solution
        v_fwd = v_new if direction == "fwd" else v_rev
        v_bwd = v_rev if direction == "fwd" else v_new
        node_seq = self.backtrack(v_fwd) + self.backtrack(v_bwd)[::-1]
        
        traj_seq = []
        prev_traj_switch = None
        for i in range(len(node_seq)-1):
            v1, v2 = node_seq[i], node_seq[i+1]
            mode, traj = self.reach_tree_edge[(v1.index, v2.index)]
            if len(traj) != 0:
                if prev_traj_switch is not None: 
                    traj = prev_traj_switch[::-1] + traj
                    prev_traj_switch = None
                if v2.traj_switch is not None: 
                    traj += v2.traj_switch
                    prev_traj_switch = v2.traj_switch
            traj_seq += [(mode, traj)]
        return node_seq, traj_seq

    def backtrack(self, node)->List[Node]:
        nodes = []
        while True:
            nodes += [node]
            if node.parent is None:
                return nodes[::-1]
            node = node.parent

    def get_mode_param(self, v_prev:Node, v_guide:Node, a:Operator):
        action_name, obj, p_del, p_add = a.name.strip("()").split(" ")
        theta_new = deepcopy(v_prev.theta)
        theta_new[obj] = deepcopy(v_guide.theta[obj])
        return theta_new

    def set_guides(self, v:Node, Gset_bwd, pi:List[Operator]):
        #set guides for backward search
        preds = deepcopy(v.state)
        state = deepcopy(v.state)
        for a in pi:
            if preds == set(): break
            if not self.is_nongeometric_action(a):
                if a.del_effects.intersection(preds) != set():
                    for pred in a.del_effects:
                        preds -= {pred}
                        pred_name, *_ = pred.strip("()").split(" ")
                        if pred_name != "attached": continue
                        G = self.get_nodeset_by_state(Gset_bwd, state)
                        obj, parent = _
                        G += [(obj, deepcopy(v.theta[obj]), v)]
            state = a.apply(state)
                    

    def sample_goal(
        self, v_init:Node, pi:List[Operator], X_goal: GoalSet, max_iter=10
    )->Node:
        raise NotImplementedError()
    
    def action_reverse(self, a:Operator)->Operator:
        if not self.is_nongeometric_action(a):
            aname, obj, parent_del, parent_add = a.name.strip("()").split(" ")
            aname_new = "pick" if aname == "place" else "place"
            name_new = " ".join([aname_new, obj, parent_add, parent_del])
        else:
            aname, obj, cond = a.name.strip("()").split(" ")
            aname_new = f"{aname}_rev"
            name_new = " ".join([aname_new, obj, cond])
        return Operator(f"({name_new})", a.add_effects, a.del_effects, a.add_effects)

    def reachability_tree_search_random_mmp(
        self, 
        states:List[frozenset], 
        pi:List[Operator],
        eps: float = 0.
    ):
        r_list = []
        nodeset = []
        
        for i, a in enumerate(pi):
            state_prev = states[i]
            nodeset_prev = self.get_nodeset_by_state(state_prev)
            # state_curr = states[i+1]
            # nodeset_curr = self.get_nodeset_by_state(state_curr)
            nodeset += [(i, node) for node in nodeset_prev]
        
        for i in range(len(pi)):
            (i, v_prev) = choose_random(nodeset)
            a = pi[i]
            state_prev = states[i]
            state_curr = states[i+1]
            nodeset_prev = self.get_nodeset_by_state(state_prev)
            nodeset_curr = self.get_nodeset_by_state(state_curr)

            if self.is_nongeometric_action(a): 
                v_new = Node(deepcopy(v_prev.q), deepcopy(v_prev.theta))
            else:
                theta_new = self.sample_mode_param(v_prev, a)
                v_new = self.sample_transition(v_prev, a, theta_new) #trick: we check ik of pre motion in it
                tau = self.get_single_mode_path(phi(states[i]), v_prev, v_new)
                if tau is None: continue
            
            r = i/(len(pi)-1)
            r_list += [r]
            
            self.add_mode_node(v_prev, v_new, state_curr)
            nodeset += [(i+1, v_new)]
            nodeset_curr += [v_new]
        
        if len(self.get_nodeset_by_state(states[-1])) != 0: 
            return 1.

        else: return np.max(r_list)
    
    def add_mode_node(
        self, 
        v_new:Node, 
        state_new:frozenset,
        v_parent:Optional[Node]=None
    ):
        index = len(self.reach_tree_node)
        v_new.parent = v_parent
        v_new.state = state_new
        v_new.index = index
        self.reach_tree_node += [v_new]
    
    def add_edge(self, v1, v2, traj):
        self.reach_tree_edge[(v1.index, v2.index)] = (Mode(v1.theta), traj)
        self.reach_tree_edge[(v2.index, v1.index)] = (Mode(v1.theta), traj[::-1])

    def sample_mode_param(
        self, v:Node, a:Operator
    )->Dict[str, Attachment]:
        raise NotImplementedError()
    
    def sample_transition(
        self, v:Node, a:Operator, theta_new:Dict[str, Attachment]
    )->Node:
        raise NotImplementedError()
    
    def get_single_mode_path(self, phi:Dict[str,str], v:Node, q_new:Config, state:frozenset):
        raise NotImplementedError()

    def get_nodeset_by_state(self, h:Dict[frozenset, List], state:frozenset)->List[Node]:
        state_key = "".join(np.sort(list(state)))
        if state_key not in h:
            h[state_key] = []
        return h[state_key]

    def backpropagate(self, node: Union[TaskNode, Node], reward: float):
        if isinstance(node, TaskNode):
            while node is not None:
                node.sum_reward += reward
                node = node.parent
        elif isinstance(node, Node):
            while node is not None:
                node.visit += 1
                node.sum_reward += reward
                node = node.parent

    def expand_new_child(self, node: TaskNode):
        np.random.shuffle(node.unexpanded_actions)
        action = node.unexpanded_actions.pop()
        node_new = self.task_tree.add_child(node, action)
        return node_new, action

    def make_terminal_node(self, node: TaskNode):
        return self.task_tree.add_child(node, "terminal")

    def get_task_plan(self, state:frozenset):
        return self.task_tree.get_task_plan(state)
    
    def select_by_uct(self, node: TaskNode, c=1, random=False):
        actions = list(node.children.keys()) #expanded actions
        np.random.shuffle(actions) #tie breaker
        ucts = []
        for a in actions:
            child = node.children[a]
            n_i = child.visit
            v_i = child.sum_reward/n_i
            uct = v_i + c * np.sqrt(2*np.log(node.visit)/n_i)
            ucts += [uct]
            #ucts += [v_i]
        # if np.random.random() < 0.2:
        if random:
            action = choose_random(actions)
        else:
            i = np.argmax(ucts)
            action = actions[i]
        return node.children[action], action
    
    def select_by_egreedy(self, node: TaskNode, eps=0.2):
        actions = list(node.children.keys()) #expanded actions
        np.random.shuffle(actions) #tie breaker
        values = []
        for a in actions:
            child = node.children[a]
            n_i = child.visit
            v_i = child.sum_reward/n_i
            values += [v_i]
        
        if np.random.random() < eps:
            i = np.argmax(values)
            action = actions[i]
        else:
            action = choose_random(actions)
        return node.children[action], action

