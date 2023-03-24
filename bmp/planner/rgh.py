from numpy import choose
from bmp import *
from bmp.planner.rrt import BiRRT2
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

@dataclass
class AbstractStateNode:
    state: frozenset
    actions: List[Operator]
    parent: Optional["AbstractStateNode"] = field(default_factory=lambda :None)
    children: Dict[Operator,"AbstractStateNode"] = field(default_factory=lambda :{})
    visit: int = field(default_factory=lambda :0)
    sum_reward: int = field(default_factory=lambda :0)
    index: int = field(default_factory=lambda : -1)
    rg_nodes: List["RGNode"] = field(default_factory=lambda :[])
    
    def sample_rg_node(self):
        if len(self.rg_nodes) != 0:
            return choose_random(self.rg_nodes)
        else:
            return None
    
    def register_rg_node(self, node:"RGNode"):
        self.rg_nodes += [node]
        node.abs_state_node = self

    @property
    def fully_expanded(self):
        return len(self.actions) == len(self.children.keys())

class AbstractStateTree:
    def __init__(self):
        self.V: List[AbstractStateNode] = []
    
    @property
    def root(self):
        return self.V[0]
    
    def add_node(self, node:AbstractStateNode):
        node.index = len(self.V)
        self.V += [node]

    def add_child(
        self, 
        parent:AbstractStateNode, 
        child:AbstractStateNode,
        action:Union[Operator, str]
    ):
        self.add_node(child)
        child.parent = parent
        parent.children[action] = child
    
    def get_tree_node_seq(self, node:AbstractStateNode, pi:List[Operator]):
        seq = [node]
        for a in pi:
            node = node.children[a]
            seq += [node]
        return seq


@dataclass
class RGNode:
    def __init__(
        self, 
        state:frozenset,
        sigma: Dict[str, Attachment], 
        q:Config, 
        traj_switch: List[Config]=None
    ):
        self.q = q
        self.sigma = sigma
        self.state = state
        
        self.index: int = -1
        self.parent: RGNode = None
        self.traj_switch = traj_switch
        self.abs_state_node = None

    def copy(self):
        q = deepcopy(self.q)
        sigma = deepcopy(self.sigma)
        state = deepcopy(self.state)
        self.abs_state_node
        node = RGNode(state, sigma, q)
        node.abs_state_node = self.abs_state_node
        return node

@dataclass
class RGEdge:
    action: Union[Operator, str]
    traj: field(default_factory=lambda :None)
    mode:Mode

# @dataclass
# class Trajectory:
#     traj:List[Config]
#     mode:Mode

class RG:
    def __init__(self):
        self.V: List[RGNode] = []
        self.E = {}
        self.sol:RGNode = None
    
    @property
    def root(self):
        return self.V[0]

    def add_node(self, node:RGNode):
        node.index = len(self.V)
        self.V += [node]
    
    def add_edge(
        self, parent:RGNode, child:RGNode, a:Operator, traj:List[Config]=None, mode=None):
        child.parent = parent
        edge = RGEdge(a, traj, mode)
        self.E[(parent.index, child.index)] = edge
    
    def get_edge(
        self, parent:RGNode, child:RGNode)->RGEdge:
        return self.E[(parent.index, child.index)]

class TaskPlanner:
    def __init__(
        self, 
        domain_file:str, 
        problem_file:str,
        eps_task:float, #epsilon greedy parameter
        eps_branch:float,
        planner="wastar",
        heuristic="hff"
    ):
        # Task Planner Setting
        self.domain_file = domain_file 
        self.problem_file = problem_file
        self.parser = Parser(
            self.domain_file, probFile=self.problem_file)
        self.domain = self.parser.parse_domain()
        self.problem = self.parser.parse_problem(self.domain)
        self.task = _ground(self.problem)

        self.state_init = self.task.initial_state
        self.search = SEARCHES[planner]
        heuristic_class = HEURISTICS[heuristic]
        self.heuristic = heuristic_class(self.task)

        # self.alpha = alpha
        self.eps_task = eps_task
        self.eps_branch = eps_branch
        # self.c = c
        self.p_terminal = 0.05
    
    def make_state_node(self, state:frozenset):
        actions = self.get_applicable_actions(state)
        return AbstractStateNode(state, actions)

    def get_applicable_actions(self, state:frozenset):
        actions = [op for op in self.task.operators if op.applicable(state)]
        result = []
        for a in actions:
            aname, *_ = a.name.strip("()").split(" ")
            if aname == "stack" or aname == "unstack":
                obj1, parent_from, parent_to = _
                if aname == "stack" and obj1 == parent_to:
                    continue
                if aname == "unstack" and obj1 == parent_from:
                    continue
            result += [a]
        #actions = [a for a in actions if a.name != ]
        return result

    def sample_action_seq(self, tree:AbstractStateTree):
        """main function"""
        node, pi_tree = self.randomized_tree_search(tree)
        pi_plan = self.get_task_plan(node.state)
        if pi_plan is None:
            print("?")
        pi = pi_tree + pi_plan
        self.extend_tree(tree, node, pi_plan)
        return pi_tree + pi_plan

    def randomized_tree_search(self, tree:AbstractStateTree):
        pi_tree = []
        node = tree.root
        
        # selection
        if np.random.random() < self.eps_task:
            while True:
                a = self.select_by_egreedy(node, terminal=False, eps=1)
                if isinstance(a, str) or len(node.children.keys()) == 0:
                    # terminate
                    break
                else:
                    node_new = node.children[a]
                    pi_tree += [a]
                    node = node_new
        else:
            while True:
                a = self.select_by_egreedy(node, self.eps_branch)
                if isinstance(a, str):
                    # terminate
                    break
                elif a not in node.children:
                    #create new node and break
                    state_new = a.apply(node.state)
                    node_new = self.make_state_node(state_new)
                    tree.add_child(node, node_new, a)
                    pi_tree += [a]
                    node = node_new
                    break
                else:
                    #select the node and continue
                    node_new = node.children[a]
                    pi_tree += [a]
                    node = node_new

        return (node, pi_tree)
    
    def select_by_egreedy(self, node: AbstractStateNode, eps, terminal=True):
        if np.random.random() < self.p_terminal and terminal:
            # terminate sequence with some probability
            return "terminal"
        
        if len(node.children) == 0 or np.random.random() > eps:
            # choose action randomly
            return choose_random(node.actions)

        else:
            # exploitation
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
       
    def get_task_plan(self, state:frozenset):
        self.task.initial_state = state
        return _search(self.task, self.search, self.heuristic)
    
    def get_task_plan_to_state(self, state:frozenset, state_goal:frozenset):
        state_goal_ = self.task.goals
        self.task.initial_state = state
        self.task.goals = state_goal
        pi = _search(self.task, self.search, self.heuristic)
        self.task.goals = state_goal_
        return pi

    def extend_tree(
        self, 
        tree:AbstractStateTree, 
        node:AbstractStateNode, 
        pi:List[Operator]
    ):
        for a in pi:
            if a in node.children:
                node = node.children[a]
            else:
                state_new = a.apply(node.state)
                node_new = self.make_state_node(state_new)
                node = tree.add_child(node, node_new, a)
                node = node_new
    
    def update_tree(
        self, 
        tree:AbstractStateTree, 
        pi:List[Operator], 
        reward:float
    ):
        node = tree.root
        for a in pi:
            node.visit += 1
            node.sum_reward += reward
            assert a in node.children, "??"
            node = node.children[a]

class GoalSet:
    def __init__(
        self,
        config = None,
        sigma = None
    ):
        self.config = config
        self.sigma = sigma
    
class MultiModalPlanner:
    def __init__(self, domain:TAMPDomain):
        self.domain = domain

    def fixed_skeleton_mmp(
        self, 
        pi:List[Operator],
        s_list: List[AbstractStateNode],
        goalset:GoalSet,
        graph:RG,
        tree:AbstractStateTree,
        to_goal=True,
        no_goal_feasibility_check=True,
    ):
        rewards = []
        s_last = s_list[-1].state
        
        # goal node sampling
        if no_goal_feasibility_check:
            v_goal = self.sample_goal_state(s_list[-1].state, goalset)
        else:
            for _ in range(10):
                v_goal = self.sample_goal_state(s_list[-1].state, goalset)
                if not self.domain.is_collision(Mode(v_goal.sigma), v_goal.q):
                    break
                v_goal = None
        if v_goal is None: return []
        indices = self.last_action_indices(pi, s_last)

        k = len(pi)

        for i, a in enumerate(pi):
            s_curr, s_next = s_list[i], s_list[i+1]
            v = s_curr.sample_rg_node()
            if v is None: break
            
            success = False
            #get param
            self.domain.assign(Mode(v.sigma), v.q)
            if not a.is_geom_action():
                state_new = a.apply(v.state)
                v_new = v.copy()
                v_new.state = state_new
                tau = None
                success = True
                
            
            else:
                if i in indices:    
                    sigma_new = self.make_adjacent_mode_by_goal(v, a, v_goal)
                else:
                    sigma_new = self.sample_adjacent_mode(v, a)
                
                
                v_new = self.sample_transition(v, a, sigma_new)
                tau = None
                if v_new is not None:
                    self.domain.assign(Mode(v.sigma), v_new.q)
                    tau = self.sample_single_mode_path(v, v_new)
                if tau is not None:
                    success = True
            
            if success:
                graph.add_node(v_new)
                graph.add_edge(v, v_new, a, tau, Mode(v.sigma))
                s_next.register_rg_node(v_new)
                success = True
            else:
                rewards += [(i)/k]
            
        if i+1 == k and success:
            tau = self.sample_single_mode_path(v_new, v_goal)
            if tau is not None:
                graph.add_node(v_goal)
                graph.add_edge(v_new, v_goal, "end", tau, Mode(v_new.sigma))
                s_next.register_rg_node(v_goal)
                rewards += [1.]
                if to_goal:
                    graph.sol = v_goal
        return rewards


    def sample_single_mode_path(self, node:RGNode, node_new:RGNode):
        if node_new is None: return None
        mode = Mode(node.sigma)
        mp = BiRRT2(q_delta_max=0.1, ts_eps=0.01)
        traj = mp.plan(
            ["robot"], node.q, node_new.q, mode, self.domain)
        if traj is not None: 
            return traj
        return None

    def sample_adjacent_mode(self, v:RGNode, a:Operator):
        obj, parent, _ = a.get_geom_obj_add_del()
        param_delta = self.domain.sample_attachment(obj, parent)
        
        mode = deepcopy(v.sigma)
        mode[obj] = param_delta

        # #for fig
        # mode_fig = deepcopy(mode)
        # for _obj in mode:
        #     mode_fig[_obj].tf = Pose(trans=[100,0,0])
        # mode_fig[obj] = param_delta
        # self.domain.assign(Mode(mode_fig))
        # self.domain.assign(Mode(mode))
        return mode

    def make_adjacent_mode_by_goal(self, v:RGNode, a:Operator, node_goal:RGNode):
        obj, parent, _ = a.get_geom_obj_add_del()
        param_delta = node_goal.sigma[obj]
        mode = deepcopy(v.sigma)
        mode[obj] = param_delta
        return mode

    def last_action_indices(self, pi:List[Operator], s_last:frozenset):
        k = len(pi)
        result = []
        for predicate in s_last:
            if not "attached" in predicate: continue
            for i, a in enumerate(pi[::-1]):
                if predicate in a.add_effects:
                    result += [k-i-1]
                    break
        return result

    def sample_transition(self, node:RGNode, a:Operator, sigma_new:Dict[str, Attachment]):
        if node is None: return None

        mode = Mode(node.sigma)
        #self.domain.assign(mode)

        obj, parent, _ = a.get_geom_obj_add_del()
        if "robot" in parent: #pick action
            grasp = sigma_new[obj]
            placement = node.sigma[obj]
            placement_parent = node.sigma[obj].parent_name
        else: #place action
            grasp = node.sigma[obj]
            placement = sigma_new[obj]
            placement_parent = parent
        
        parent_pose = self.domain.objects[placement_parent].get_base_pose()
        obj_pose = parent_pose * placement.tf.inverse()
        grasp_pose = obj_pose * grasp.tf
        grasp_pose_pre = grasp.get_pre_pose(grasp_pose)
        
        robot = self.domain.robots["robot"]
        q_rand = robot.get_random_joint_angles()
        robot.set_joint_angles(q_rand)
        q_pre_ik = robot.inverse_kinematics(pose=grasp_pose_pre)
        if q_pre_ik is None: return None
        
        config_pre = deepcopy(node.q)
        config_pre.set_q("robot", q_pre_ik)
        state_new = a.apply(node.state)
        if self.domain.is_collision(Mode(sigma_new), config_pre): 
            return None
        
        mp = BiRRT2(ts_eps=0.02)
        traj_switch = mp.check_mode_switch(
            ["robot"], config_pre, grasp_pose, mode, self.domain)
        if traj_switch is not None: 
            return RGNode(state_new, sigma_new, config_pre, traj_switch=traj_switch)
        return None
    
    def sample_goal_state(
        self, 
        last_abs_state:frozenset,
        goalset:GoalSet
    )->RGNode:
        #config
        if goalset.config is not None:
            config_goal = goalset.config
        else:
            q = self.domain.robots["robot"].get_random_joint_angles()
            config_goal = Config({"robot":q})
        
        theta_goal = {}
        for predicate in last_abs_state:
            if not "attached" in predicate: continue
            _, obj, parent = predicate.strip("()").split(" ")
            
            if goalset.sigma is None:
                param = self.domain.sample_attachment(obj, parent)
            elif obj not in goalset.sigma:
                param = self.domain.sample_attachment(obj, parent)
            else:
                param = deepcopy(goalset.sigma[obj])
            theta_goal[obj] = param
        
        return RGNode(last_abs_state, theta_goal, config_goal)

class RGH:
    def __init__(
        self,
        domain_file:str,
        problem_file:str,
        domain:TAMPDomain,
        problem:TAMPProblem,
        use_reward = True,
        planner = "gbf",
        heuristic = "hff",
        mmp_iter = 2,
        eps_task = 0.2,
        eps_branch = 0.5,
        no_goal_sampling=False,
    ):
        self.domain_file = domain_file
        self.problem_file = problem_file
        self.domain = domain
        self.problem = problem
        self.use_reward = use_reward
        self.planner = planner
        self.heuristic = heuristic
        self.mmp_iter = mmp_iter
        self.eps_task = eps_task
        self.eps_branch = eps_branch
        self.no_goal_sampling=no_goal_sampling

    def init(self):
        self.tp  = TaskPlanner(
            self.domain_file, 
            self.problem_file, 
            eps_task=self.eps_task,
            eps_branch=self.eps_branch,
            planner=self.planner, 
            heuristic=self.heuristic
        )
        self.mmp = MultiModalPlanner(self.domain)
        self.mp = BiRRT2()

        state_init = self.tp.state_init
        sigma_init = self.problem.mode_init.attachments
        config_init = self.problem.config_init
        if isinstance(self.problem.mode_goal, Mode):
            sigma_goal = self.problem.mode_goal.attachments
        else:
            sigma_goal = self.problem.mode_goal

        goalset = GoalSet(config=config_init, sigma=sigma_goal)
        rg_root = RGNode(state_init, sigma_init, config_init)
        tree_root = self.tp.make_state_node(state_init)

        self.tree = AbstractStateTree()
        self.graph = RG()
        self.tree.add_node(tree_root)
        self.graph.add_node(rg_root)
        
        tree_root.register_rg_node(rg_root)
        return self.tree, self.graph, goalset


    def plan(self, timeout=300):
        tree, graph, goalset = self.init()
        start = time.time()
        while time.time() - start < timeout:
            rewards = []
            pi = self.tp.sample_action_seq(tree)
            s_list = tree.get_tree_node_seq(tree.root, pi)
            if len(s_list) == 1: continue
            for _ in range(self.mmp_iter):
                rewards += self.mmp.fixed_skeleton_mmp(pi, s_list, goalset, graph, tree, no_goal_feasibility_check=self.no_goal_sampling)
            if self.use_reward:
                self.tp.update_tree(tree, pi, np.mean(rewards))

            if graph.sol is not None:
                print("goal")
                return

    def visualize(self):
        node = self.graph.sol
        commands: List[RGEdge] = []
        nodes: List[RGNode] = [node]
        
        while True:
            if node.parent is None: break
            command = self.graph.get_edge(node.parent, node)    
            
            commands += [command]
            nodes += [node.parent]
            node = node.parent
        nodes = nodes[::-1]
        commands = commands[::-1]

        mp = BiRRT2()
        for i, command in enumerate(commands[:-1]):
            if commands[i].traj is not None: #geometric
                if nodes[i+1].traj_switch is not None:
                    traj_switch_pu = nodes[i+1].traj_switch
                    commands[i].traj = commands[i].traj + traj_switch_pu
                elif commands[i+1].traj is None and nodes[i+2].traj_switch is not None:
                    traj_switch_pu = nodes[i+2].traj_switch
                    commands[i].traj = commands[i].traj + traj_switch_pu

                if nodes[i].traj_switch is not None:
                    commands[i].traj = nodes[i].traj_switch[::-1] + commands[i].traj
                elif nodes[i-1].traj_switch is not None:
                    commands[i].traj = nodes[i-1].traj_switch[::-1] + commands[i].traj
                
                mp.init(["robot"], self.domain, commands[i].mode)
                commands[i].traj = mp.smoothing(commands[i].traj, max_iter=25)
        
        #for last traj
        if nodes[-2].traj_switch is not None:
            commands[-1].traj = nodes[-2].traj_switch[::-1] + commands[-1].traj
        elif nodes[-3].traj_switch is not None:
            commands[-1].traj = nodes[-3].traj_switch[::-1] + commands[-1].traj
        mp.init(["robot"], self.domain, commands[i].mode)
        commands[-1].traj = mp.smoothing(commands[-1].traj, max_iter=25)
        
        # show
        for command in commands:
            if command.traj is not None:
                for config in command.traj:
                    self.domain.assign(command.mode, config)
                    time.sleep(0.05)
            else:
                # non-geometric action
                pass

class RGH_rp(RGH):
    def sample_state(self):
        movables = list(self.domain.movables.keys())
        parents = list(self.domain.placeables.keys())
        
        state = []
        robot = False
        attached = []
        parents += ["robot"]
        np.random.shuffle(movables)
        for movable in movables:
            while True:
                parent = choose_random(parents)
                if parent != movable:
                    break
            parents.remove(parent)
            state += [f"(attached {movable} {parent})"]
        return frozenset(state)

    def sample_mode(self, abs_attachments):
        atts = {}
        for abs_att in abs_attachments:
            _, obj, parent = abs_attachments.strip("()").split(" ")
            att = self.domain.sample_attachment(obj, parent)
            atts[obj] = att
        return Mode(atts)

    def nearest(self, graph:RG):
        for node in graph.V:
            mode = Mode(node.sigma)
            d = mode.as_1d_numpy()
            print(d)

    def plan(self, eps = 0.2, timeout=300):
        tree, graph, goalset = self.init()
        start = time.time()
        while time.time() - start < timeout:
            
            rg_node = choose_random(self.graph.V)
            s_node = rg_node.abs_state_node
            
            if np.random.random() > eps:
                #exploration
                s_goal = self.sample_state()
                node_goal = self.mmp.sample_goal_state(s_goal, goalset)    
                pi = self.tp.get_task_plan_to_state(rg_node.state, s_goal)
                to_goal = False
            else:
                #exploitation
                pi = self.tp.get_task_plan(rg_node.state)
                to_goal = True
            if pi is None: continue
            self.tp.extend_tree(tree, s_node, pi)
            s_list = tree.get_tree_node_seq(s_node, pi)

            for _ in range(1):
                self.mmp.fixed_skeleton_mmp(pi, s_list, goalset, graph, tree, to_goal)

            if graph.sol is not None:
                print("goal")
                return
        
if __name__ == "__main__":
    dom = DomainKitchen(gui=True, num_box=5)
    prob = ProblemKitchen(dom, num_block=5, cooked_list=[1, 2, 3, 4, 5])
    domain_file = Path(__file__).parent.parent / "domain/kitchen/pddl/domain.pddl"
    problem_file = Path(__file__).parent.parent / "domain/kitchen/pddl/problem_box5.pddl"
    
    rghrp = RGH(domain_file, problem_file, dom, prob, planner="wastar", heuristic="hff")
    start = time.perf_counter()
    rghrp.plan()
    end = time.perf_counter()
    print(f"elapsed:{end-start}")
    rghrp.visualize()
    
    

    

    

    
            
    
    
