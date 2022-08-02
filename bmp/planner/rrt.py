import numpy as np
from pybullet_suite import *
from typing import Callable
from ..base import *

# class Node:
#     """Configuration of robots
#     """
#     def __init__(
#         self, 
#         q: np.ndarray, 
#         T: Optional[Pose]=None, 
#     ):
#         self.q = q
#         self.T = T
#         self.index = -1 # not assigned

#     def copy(self) -> "Node":
#         config = deepcopy(self)
#         config.index = -1
#         return config

def distance_ts(
    T1: Pose, 
    T2: Pose, 
    rot_weight=0.5
) -> float:
    linear = np.linalg.norm(T1.trans - T2.trans)
    qtn1, qtn2 = T1.rot.as_quat(), T2.rot.as_quat()
    if qtn1 @ qtn2 < 0:
        qtn2 = -qtn2
    angular = np.arccos(np.clip(qtn1 @ qtn2, -1, 1))
    #print(f"distance - linear: {linear} angular: {angular}")
    return linear + rot_weight * angular

# class Tree:
#     def __init__(self, root: Node):
#         root.index = 0
#         self.root = root
#         self.data = [root]
#         self.parent = {0:-1}
#         self.num = 1

#     def add_node(self, node: Node, parent: Node):
#         assert node.index != parent.index
#         node.index = self.num
#         self.parent[node.index] = parent.index
#         self.data.append(node)
#         self.num += 1
    
#     def nearest(self, node: Node) -> Node:
#         distances = []
#         for node_tree in self.data:
#             d = np.linalg.norm(node_tree.q - node.q)
#             distances.append(d)
#         idx = np.argmin(distances)
#         return self.data[idx]
    
#     def nearest_tspace(self, T: Pose)->Node:
#         distances = []
#         for node in self.data:
#             d = distance_ts(node.T, T)
#             distances.append(d)
#         return self.data[np.argmin(distances)]

#     def backtrack(self, node: Node):
#         path = []
#         node_curr = node
#         while True:
#             path.append(node_curr)
#             parent_index = self.parent[node_curr.index]
#             if parent_index == -1:
#                 break
#             node_curr = self.data[parent_index]
#         return path[::-1]

# class BiRRT:
#     def __init__(
#         self,
#         eps: float = 0.2,
#         p_goal: float = 0.2,
#         max_iter: int = 100,
#         q_delta_max: float = 0.1,
#         DLS_damping: float = 0.1,
#     ):
#         self.eps = eps
#         self.p_goal = p_goal
#         self.max_iter = max_iter
#         self.q_delta_max = q_delta_max
#         self.DLS_damping = DLS_damping
#         self.is_goal = lambda node: self.distance(node, self.goal) < self.eps
    
#     def get_random_node(self):
#         robot: Panda = self.kingraph.objects[self.robot_name]
#         return Node(robot.get_random_arm_angles())

#     def plan(self, robot_name: str, config_init: Config, start: np.ndarray, goal: np.ndarray, kingraph: KinGraph):
#         self.robot_name = robot_name
#         self.config = config_init
#         self.init(Node(start), Node(goal), kingraph)
        
#         tree_a = self.tree_start
#         tree_b = self.tree_goal
#         for i in range(self.max_iter):
#             node_rand = self.get_random_node()
            
#             if not self.extend(tree_a, node_rand) == "trapped":
#                 if self.connect(tree_b, self._node_new) == "reached":
#                     return self.get_path()
#             (tree_a, tree_b) = (tree_b, tree_a)
#         return []

#     def backtrack(self, tree: Tree, last_node: Node) -> List[Config]:
#         traj_ = tree.backtrack(last_node)
#         traj = []
#         for node in traj_:
#             config = self.config.copy()
#             config.q[self.robot_name] = node.q
#             traj.append(config)
#         return traj

#     def init(self, start: Node, goal: Node, kingraph: KinGraph):
#         self.start = start
#         self.goal = goal
#         self.kingraph = kingraph
#         self.tree_start = Tree(start)
#         self.tree_goal = Tree(goal)

#     def connect(self, tree, node):
#         result = "advanced"
#         while result == "advanced":
#             result = self.extend(tree, node)
#         return result

#     def distance(self, node1:Node, node2:Node):
#         return np.linalg.norm(node1.q - node2.q)

#     def extend(self, tree: Tree, node_rand: Node):
#         node_near = tree.nearest(node_rand)
#         node_new = self.control(node_near, node_rand)
#         if node_new is not None:
#             tree.add_node(node_new, node_near)
#             if not self.distance(node_rand, node_new) > self.eps:
#                 self.last_node = node_new
#                 return "reached"
#             else:
#                 self._node_new = node_new #save this to "connect"
#             return "advanced"
#         return "trapped"
    
#     def limit_step_size(self, q_delta: np.ndarray, q_delta_max: Optional[np.ndarray]=None):
#         if q_delta_max is None:
#             q_delta_max = self.q_delta_max
#         mag = np.linalg.norm(q_delta, np.inf)
#         if mag > q_delta_max:
#             q_delta = q_delta / mag * q_delta_max
#         return q_delta

#     def control(self, node_near:Node, node_rand:Node):
#         mag = self.distance(node_near, node_rand)
#         if mag <= self.eps:
#             node_new = node_rand.copy()
#             node_new.index = -1
#         else:
#             q_err = node_rand.q - node_near.q
#             q_delta = self.limit_step_size(q_err, self.q_delta_max)
#             q_new = node_near.q + q_delta
#             node_new = Node(q_new)
        
#         config_new = self.config.copy()
#         config_new.q[self.robot_name] = q_new
#         if not self.kingraph.is_collision(config_new):
#             return node_new
#         else:
#             return None
    
#     def get_path(self):
#         node_tree_start = self.tree_start.nearest(self.last_node)
#         node_tree_goal = self.tree_goal.nearest(self.last_node)
#         path_from_start = self.backtrack(self.tree_start, node_tree_start)
#         path_from_goal = self.backtrack(self.tree_goal, node_tree_goal)
        
#         return [*path_from_start, *path_from_goal[::-1]]

# class Node3:
#     """Configuration of robots
#     """
#     def __init__(
#         self, 
#         q_dict: dict
#     ):
#         self.q_dict = q_dict
#         self.index = -1 # not assigned

#     @property
#     def q(self):
#         qs = []
#         for robot in ["robot1", "robot2"]:
#             qs.append(self.q_dict[robot])
#         return np.hstack(qs)

#     def copy(self) -> "Node3":
#         config = deepcopy(self)
#         config.index = -1
#         return config
    
#     @classmethod
#     def from_array(cls, q: np.ndarray):
#         q_dict = {}
#         q_dict["robot1"] = q[:7]
#         q_dict["robot2"] = q[7:]
#         return cls(q_dict)

# class Tree3:
#     def __init__(self, root: Node3):
#         root.index = 0
#         self.root = root
#         self.data = [root]
#         self.parent = {0:-1}
#         self.num = 1

#     def add_node(self, node: Node3, parent: Node3):
#         assert node.index != parent.index
#         node.index = self.num
#         self.parent[node.index] = parent.index
#         self.data.append(node)
#         self.num += 1
    
#     def nearest(self, node: Node3) -> Node3:
#         distances = []
#         for node_tree in self.data:
#             d = np.linalg.norm(node_tree.q - node.q)
#             distances.append(d)
#         idx = np.argmin(distances)
#         return self.data[idx]

#     def backtrack(self, node: Node):
#         path = []
#         node_curr = node
#         while True:
#             path.append(node_curr)
#             parent_index = self.parent[node_curr.index]
#             if parent_index == -1:
#                 break
#             node_curr = self.data[parent_index]
#         return path[::-1]


# class BiRRT3:
#     def __init__(
#         self,
#         eps: float = 0.2,
#         p_goal: float = 0.2,
#         max_iter: int = 100,
#         q_delta_max: float = 0.1,
#         DLS_damping: float = 0.1,
#     ):
#         self.eps = eps
#         self.p_goal = p_goal
#         self.max_iter = max_iter
#         self.q_delta_max = q_delta_max
#         self.DLS_damping = DLS_damping
#         self.is_goal = lambda node: self.distance(node, self.goal) < self.eps
    
#     def get_random_node(self):
#         qs = {}
#         for robot_name in self.robot_names:
#             robot: Panda = self.kingraph.objects[robot_name]
#             qs[robot_name] = robot.get_random_arm_angles()
#         return Node3(qs)

#     def plan(self, robot_names: List[str], config_init: Config, config_end: Config, kingraph: KinGraph):
#         self.robot_names = robot_names
#         self.config = config_init
#         self.init(Node3(config_init.q), Node3(config_end.q), kingraph)
        
#         tree_a = self.tree_start
#         tree_b = self.tree_goal
#         for i in range(self.max_iter):
#             node_rand = self.get_random_node()
            
#             if not self.extend(tree_a, node_rand) == "trapped":
#                 if self.connect(tree_b, self._node_new) == "reached":
#                     return self.get_path()
#             (tree_a, tree_b) = (tree_b, tree_a)
#         return []

#     def backtrack(self, tree: Tree, last_node: Node) -> List[Config]:
#         traj_ = tree.backtrack(last_node)
#         traj = []
#         for node in traj_:
#             config = self.config.copy()
#             for robot_name in self.robot_names:
#                 config.q[robot_name] = node.q_dict[robot_name]
#             traj.append(config)
#         return traj

#     def init(self, start: Node3, goal: Node3, kingraph: KinGraph):
#         self.start = start
#         self.goal = goal
#         self.kingraph = kingraph
#         self.tree_start = Tree3(start)
#         self.tree_goal = Tree3(goal)

#     def connect(self, tree, node):
#         result = "advanced"
#         while result == "advanced":
#             result = self.extend(tree, node)
#         return result

#     def distance(self, node1:Node3, node2:Node3):
#         return np.linalg.norm(node1.q - node2.q)

#     def extend(self, tree: Tree, node_rand: Node3):
#         node_near = tree.nearest(node_rand)
#         node_new = self.control(node_near, node_rand)
#         if node_new is not None:
#             tree.add_node(node_new, node_near)
#             if not self.distance(node_rand, node_new) > self.eps:
#                 self.last_node = node_new
#                 return "reached"
#             else:
#                 self._node_new = node_new #save this to "connect"
#             return "advanced"
#         return "trapped"
    
#     def limit_step_size(self, q_delta: np.ndarray, q_delta_max: Optional[np.ndarray]=None):
#         if q_delta_max is None:
#             q_delta_max = self.q_delta_max
#         mag = np.linalg.norm(q_delta, np.inf)
#         if mag > q_delta_max:
#             q_delta = q_delta / mag * q_delta_max
#         return q_delta

#     def control(self, node_near:Node3, node_rand:Node3):
#         mag = self.distance(node_near, node_rand)
#         if mag <= self.eps:
#             node_new = node_rand.copy()
#             node_new.index = -1
#         else:
#             q_err = node_rand.q - node_near.q
#             q_delta = self.limit_step_size(q_err, self.q_delta_max)
#             q_new = node_near.q + q_delta
#             node_new = Node3.from_array(q_new)
        
#         config_new = self.config.copy()
#         for robot_name in self.robot_names:
#             config_new.q[robot_name] = node_new.q_dict[robot_name]
#         if not self.kingraph.is_collision(config_new):
#             return node_new
#         else:
#             return None
    
#     def get_path(self):
#         node_tree_start = self.tree_start.nearest(self.last_node)
#         node_tree_goal = self.tree_goal.nearest(self.last_node)
#         path_from_start = self.backtrack(self.tree_start, node_tree_start)
#         path_from_goal = self.backtrack(self.tree_goal, node_tree_goal)
        
#         return [*path_from_start, *path_from_goal[::-1]]



# class TSRRT:
#     def __init__(
#         self,
#         eps: float = 0.2,
#         p_goal: float = 0.2,
#         max_iter: int = 100,
#         q_delta_max: float = 0.1,
#         DLS_damping: float = 0.1,
#     ):
#         self.eps = eps
#         self.p_goal = p_goal
#         self.max_iter = max_iter
#         self.q_delta_max = q_delta_max
#         self.DLS_damping = DLS_damping
        
#     def check_mode_switch(
#         self, 
#         robot_name: str,
#         node_final: Config, 
#         T_target: Pose,
#         kingraph: KinGraph
#     ):
#         EPS = 0.01
#         result = None
#         traj = []
#         robot: Panda = kingraph.objects[robot_name]
#         with robot.no_set_joint():
#             config = node_final.copy()
#             for _ in range(10):
#                 q_old = config.q[robot_name]
#                 q = self.steer(
#                     q=q_old,
#                     jac=robot.get_jacobian(q_old),
#                     curr_pose=robot.forward_kinematics(q_old),
#                     target_pose=T_target,
#                     q_delta_max=0.05
#                 )
#                 config_new = node_final.copy()
#                 config_new.q[robot_name] = q
#                 robot.set_joint_angles(q)
#                 T = robot.get_ee_pose()
#                 kingraph.assign()
#                 if not kingraph.is_collision(config_new):
#                     traj.append(config_new)
#                     if distance_ts(T, T_target) < EPS:
#                         result = traj
#                         break
#                     config = config_new
#                 else:
#                     break
                
#         kingraph.assign()
#         return result
    
#     def steer(
#         self,
#         q: np.ndarray,
#         jac: np.ndarray,
#         curr_pose: Pose,
#         target_pose: Pose,
#         q_delta_max: Optional[float] = None
#     ) -> np.ndarray:
#         if q_delta_max is None:
#             q_delta_max = self.q_delta_max
#         pos_err = target_pose.trans - curr_pose.trans
#         orn_err = orn_error(target_pose.rot.as_quat(), curr_pose.rot.as_quat())
#         err = np.hstack([pos_err, orn_err*2])
#         lmbda = np.eye(6) * self.DLS_damping ** 2
#         jac_pinv = jac.T @ np.linalg.inv(jac @ jac.T + lmbda)
#         q_delta = self.limit_step_size(jac_pinv @ err, q_delta_max)
#         return q_delta + q

#     def limit_step_size(self, q_delta: np.ndarray, q_delta_max: Optional[np.ndarray]=None):
#         if q_delta_max is None:
#             q_delta_max = self.q_delta_max
#         mag = np.linalg.norm(q_delta, np.inf)
#         if mag > q_delta_max:
#             q_delta = q_delta / mag * q_delta_max
#         return q_delta

class Node2(Config):
    """Configuration of robots
    """
    def __init__(
        self,
        q: Dict[str, np.ndarray],
        T: Dict[str, np.ndarray] = {}
    ):
        self.q = q
        self.T = T
        self.index = None
        self.tcp_dist = None
        self.excluded = False

    def assign_numpy(self, q):
        assert len(q) == len(self.q_numpy)
        idx_start = 0
        for robot_name in self.q:
            q_len = len(self.q[robot_name])
            self.q[robot_name] = q[idx_start:idx_start+q_len]
            idx_start = q_len

    def __add__(self, other):
        result = self.copy()
        for robot_name in self.q:
            result.q[robot_name] += other.q[robot_name]
        return result
    
    def __sub__(self, other):
        result = self.copy()
        for robot_name in self.q:
            result.q[robot_name] -= other.q[robot_name]
        return result

    def __mul__(self, other: float):
        result = self.copy()
        for robot_name in self.q:
            result.q[robot_name] *= other
        return result

    def __truediv__(self, other: float):
        result = self.copy()
        for robot_name in self.q:
            result.q[robot_name] /= other
        return result

    @property
    def q_numpy(self) -> np.ndarray:
        qs = []
        for robot_name in self.q:
            qs.append(self.q[robot_name])
        return np.hstack(qs)

    def norm(self, *args):
        return np.linalg.norm(self.q_numpy, *args)

    def __eq__(self, other: "Node2") -> bool:
        for robot_name in self.q:
            if not np.array_equal(self.q[robot_name], other.q[robot_name]):
                return False
        return True

    def copy(self) -> "Node2":
        q_new = deepcopy(self.q)
        return Node2(q_new)

class Tree2:
    """Tree structure for TS-RRT
    """
    def __init__(
        self, 
        root: Optional[Node2] = None, 
    ):
        self.V: List[Node2] = []
        self.parent = {}
        if root is not None:
            self.add_root(root)
        self.temp_parent: Optional[Node2] = None
    
    def __len__(self):
        return len(self.V)
    
    def add_root(self, root: Node2):
        root.index = len(self.V)
        self.V.append(root)
        self.parent[root.index] = -1
    
    def add_node(self, node: Node2, parent: Node2):
        assert parent.index is not None
        node.index = len(self.V)
        self.V.append(node)
        self.parent[node.index] = parent.index
    
    def nearest_cs(self, node_rand: Node2) -> Node2:
        distances = []
        for node in self.V:
            #q_rand = node_rand.q_numpy()
            #q = node.q_numpy
            d = (node_rand - node).norm()
            distances.append(d)
        return self.V[np.argmin(distances)]
    
    def nearest_tcp_dist(self) -> Node2:
        distances = []
        for node in self.V:
            d = node.tcp_dist if node.excluded == False else np.inf
            distances.append(d)
        return self.V[np.argmin(distances)]

    def backtrack(self, node: Node2):
        path = [node]
        parent_idx = self.parent[node.index]
        while True:
            if parent_idx == -1:
                break
            path.append(self.V[parent_idx])
            parent_idx = self.parent[parent_idx]
        return path[::-1]

class RRTBase:
    def __init__(
        self,
        eps: float = 0.2,
        p_goal: float = 0.2,
        max_iter: int = 100,
        q_delta_max: float = 0.1,
        DLS_damping: float = 0.1
    ):
        self.eps = eps
        self.p_goal = p_goal
        self.max_iter = max_iter
        self.q_delta_max = q_delta_max
        self.DLS_damping = DLS_damping
        self.is_goal = lambda node: self.distance(node, self.goal) < self.eps

    def init(
        self,
        robot_names: List[str],
        domain: TAMPDomain,
        mode: Mode,
    ):
        self.domain = domain
        self.robots: Dict[str, Panda] = {}
        for robot in robot_names:
            self.robots[robot] = self.domain.robots[robot]
        self.mode = mode
    
    def get_random_node(self) -> Node2:
        q = {}
        for robot_name in self.robots:
            value = self.robots[robot_name].get_random_arm_angles()
            q[robot_name] = value
        return Node2(q)
    
    def control(self, node_near:Node2, node_rand:Node2):
        node_err = node_rand - node_near
        node_delta = self.limit_step_size(node_err, self.q_delta_max)
        node_new = node_near + node_delta
        #node_new = node_near.copy()
        #node_new.assign_numpy(q_new)
        
        with self.domain.world.no_rendering():
            if not self.domain.is_collision(self.mode, Config(node_new.q)):
                self.get_FK(node_new)
                return node_new
            else:
                return None
            
    def distance(self, node1:Node2, node2:Node2):
        return (node1 - node2).norm()
        #np.linalg.norm(node1.q_numpy - node2.q_numpy)
    
    def limit_step_size(self, node_delta: Node2, q_delta_max: Optional[np.ndarray]=None):
        if q_delta_max is None:
            q_delta_max = self.q_delta_max
        mag = node_delta.norm(np.inf) # np.linalg.norm(q_delta.q_numpy, np.inf)
        if mag > q_delta_max:
            node_delta = node_delta / mag * q_delta_max
        return node_delta
    
    def get_FK(self, node: Node2):
        tcp = []
        for robot_name, robot in self.robots.items():
            q = node.q[robot_name]
            node.T[robot_name] = robot.forward_kinematics(q)
            tcp.append(node.T[robot_name].trans)
        
        if len(tcp) == 2:
            node.tcp_dist = np.linalg.norm(tcp[0] - tcp[1])

class BiRRT2(RRTBase):
    """Dual arm RRT

    Args:
        RRTBase (_type_): _description_
    """
    def __init__(
        self,
        eps: float = 0.2,
        p_goal: float = 0.2,
        max_iter: int = 100,
        q_delta_max: float = 0.4,
        DLS_damping: float = 0.1,
    ):
        super().__init__(eps, p_goal, max_iter, q_delta_max, DLS_damping)
        

    def plan(
        self, 
        target_robot_name: List[str], 
        config_init: Config, 
        config_goal: Config,
        mode: Mode,
        domain: TAMPDomain
    ):
        self.init(target_robot_name, domain, mode)

        self.config_init = config_init
        self.config_goal = config_goal
        root_start = Node2(self.config_init.q)
        root_goal = Node2(self.config_goal.q)
        self.tree_start = Tree2(root_start)
        self.tree_goal = Tree2(root_goal)
        
        tree_a = self.tree_start
        tree_b = self.tree_goal
        for i in range(self.max_iter):
            node_rand = self.get_random_node()
            if not self.extend(tree_a, node_rand) == "trapped":
                if self.connect(tree_b, tree_a.prev_node) == "reached":
                    forward_path = self.tree_start.backtrack(self.tree_start.prev_node)
                    backward_path =self.tree_goal.backtrack(self.tree_goal.prev_node)[::-1]
                    path = [*forward_path, *backward_path]
                    return [Config(node.q) for node in path]
            (tree_a, tree_b) = (tree_b, tree_a)
        return []

    def extend(self, tree: Tree2, node_rand: Node2):
        tree.prev_node = None
        node_near = tree.nearest_cs(node_rand)
        node_new = self.control(node_near, node_rand)
        if node_new is not None:
            if not self.distance(node_rand, node_new) > self.eps:
                tree.prev_node = node_near
                return "reached"
            tree.add_node(node_new, node_near)
            tree.prev_node = node_near
            return "advanced"
        return "trapped"
    

    def connect(self, tree, node):
        result = "advanced"
        while result == "advanced":
            result = self.extend(tree, node)
        return result


# class BiIKRRT(RRTBase):
#     """ Planner for Handover
#     """
#     def __init__(
#         self,
#         eps: float = 0.2,
#         p_goal: float = 0.2,
#         max_iter: int = 100,
#         q_delta_max: float = 0.1,
#         DLS_damping: float = 0.1,
#     ):
#         super().__init__()
#         self.eps = eps
#         self.p_goal = p_goal
#         self.max_iter = max_iter
#         self.q_delta_max = q_delta_max
#         self.DLS_damping = DLS_damping
#         #self.is_goal = lambda node: self.distance(node, self.goal) < self.eps

#     def plan(
#         self, 
#         robot_name: str, 
#         robot2_name: str, 
#         config_init: Config, 
#         kingraph: KinGraph, 
#         fix_grasp: Optional[Grasp]=None, 
#         grasp_sampler = None
#     ):
#         self.kingraph = kingraph
#         self.robot_name = robot_name #holding object
#         self.robot2_name = robot2_name #handover robot
#         self.robots : Dict[str, Panda]= {name:self.kingraph.objects[name] for name in self.kingraph.robots}
#         self.config_init = config_init
#         root_start = Node2(self.config_init.q)
#         self.get_FK(root_start)
#         self.tree_start = Tree2(root_start)
#         self.tree_goal = Tree2() #no configuration
#         obj_name = kingraph.relation.child[robot_name]
#         self.grasp_holding: Grasp = kingraph.kin_edge[obj_name]
#         self.fix_grasp = fix_grasp
#         self.grasp_sampler = grasp_sampler
#         self.last_node = []
#         self.node_new = None
        
#         tree1, tree2 = self.tree_goal, self.tree_start
#         for _ in range(self.max_iter):
#             tree1, tree2 = (tree2, tree1)
#             node_rand = self.get_random_node()
#             result1 = self.extend(tree1, node_rand)
#             if (result1 == "trapped") | (result1 == "empty"): continue
#             self.kingraph.assign(Config(tree1.prev_node.q))
#             result2 = self.connect(tree2, tree1.prev_node)
#             if (result1 == "advanced") & (result2 == "reached"):
#                 #solution
#                 forward_path = self.tree_start.backtrack(self.tree_start.prev_node)
#                 backward_path =self.tree_goal.backtrack(self.tree_goal.prev_node)[::-1]
#                 path = [*forward_path, *backward_path]
#                 return [Config(node.q) for node in path], path[-1].obj_handover_pose
#                 #path, handover pose
            
#             if (len(self.tree_goal)==0) | (np.random.random() < 0.5):
#                 #make IK pose to tree_goal using tree_start
#                 node_tree = self.tree_start.nearest_tcp_dist()
#                 node_tree.excluded = True
#                 node_ik = self.compute_IK(node_tree)
#                 if node_ik is None: continue
#                 if not self.kingraph.is_collision(Config(node_ik.q)):
#                     self.tree_goal.add_root(node_ik)
            
#         return None
    

#     def compute_IK(self, node_tree: Node2):
#         movable_name = self.grasp_holding.obj_name
#         if self.fix_grasp is None:
#             #grasp_handover = self.kingraph.sample_grasp(movable_name, handover_robot_name)
#             grasp_handover: Grasp = self.grasp_sampler.sample_grasp_bunch(movable_name, self.robot2_name, self.kingraph, cond_grasp=self.grasp_holding, k=5)
#         else:
#             grasp_handover: Grasp = self.fix_grasp
        
#         #pose target
#         tcp_holding = node_tree.T[self.robot_name]
#         tcp_handover = node_tree.T[self.robot2_name]
#         obj_pose_holding = tcp_holding * self.grasp_holding.tf.inverse()
#         obj_pose_handover = tcp_handover * grasp_handover.get_pre_pose(grasp_handover.tf).inverse()
#         pos_mid = (obj_pose_holding.trans + obj_pose_handover.trans)/2
#         orn_mid = slerp(obj_pose_holding.rot.as_quat(), obj_pose_handover.rot.as_quat(), 0.5)
#         obj_pose_mid = Pose(Rotation.from_quat(orn_mid), pos_mid)
        
#         grasp_pose_robot1 = obj_pose_mid * self.grasp_holding.tf
#         q1 = self.robots[self.robot_name].inverse_kinematics(pose=grasp_pose_robot1)
#         if q1 is None: return None
#         grasp_pose_robot2 = grasp_handover.get_pre_pose(obj_pose_mid * grasp_handover.tf)
#         q2 = self.robots[self.robot2_name].inverse_kinematics(pose=grasp_pose_robot2)
#         if q2 is None: return None
#         node_ik = Node2(
#             q={self.robot_name:q1, self.robot2_name:q2},
#             T={self.robot_name:grasp_pose_robot1, self.robot2_name:grasp_pose_robot2}
#         )
#         node_ik.tcp_dist = np.linalg.norm(grasp_pose_robot1.trans - grasp_pose_robot2.trans)
#         node_ik.obj_handover_pose = obj_pose_mid
#         return node_ik
        


#     def connect(self, tree: Tree2, node: Node2):
#         result = "advanced"
#         while result == "advanced":
#             result = self.extend(tree, node)
#         return result

#     def extend(self, tree: Tree2, node_rand: Node2):
#         if len(tree) == 0:
#             return "empty"
#         node_near = tree.nearest_cs(node_rand)
#         node_new = self.control(node_near, node_rand)
#         if node_new is not None:
#             if not self.distance(node_rand, node_new) > self.eps:
#                 tree.prev_node = node_near
#                 return "reached"
#             tree.add_node(node_new, node_near)
#             tree.prev_node = node_new
#             return "advanced"
#         return "trapped"
    
#     # def control(self, node_near:Node2, node_rand:Node2):
#     #     q_err = node_rand.q_numpy - node_near.q_numpy
#     #     q_delta = self.limit_step_size(q_err, self.q_delta_max)
#     #     q_new = node_near.q_numpy + q_delta

#     #     node_new = node_near.copy()
#     #     node_new.assign_numpy(q_new)
        
#     #     if not self.kingraph.is_collision(Config(node_new.q)):
#     #         self.get_FK(node_new)
#     #         return node_new
#     #     else:
#     #         return None
    
#     # def limit_step_size(self, q_delta: np.ndarray, q_delta_max: Optional[np.ndarray]=None):
#     #     if q_delta_max is None:
#     #         q_delta_max = self.q_delta_max
#     #     mag = np.linalg.norm(q_delta, np.inf)
#     #     if mag > q_delta_max:
#     #         q_delta = q_delta / mag * q_delta_max
#     #     return q_delta

    

#     # def distance(self, node1:Node2, node2:Node2):
#     #     return np.linalg.norm(node1.q_numpy - node2.q_numpy)


# class JinvRRT:
#     def __init__(
#         self,
#         eps: float = 0.1,
#         p_global_explore: float = 0.5,
#         p_constraint_explore: float = 0.5,
#         max_iter: int = 100,
#         q_delta_max: float = 0.2,
#         DLS_damping: float = 0.1,
#         eps_ts: float = 0.05
#     ):
#         self.eps = eps
#         self.p_global_explore = p_global_explore
#         self.p_constraint_explore = p_constraint_explore
#         self.max_iter = max_iter
#         self.q_delta_max = q_delta_max
#         self.DLS_damping = DLS_damping
#         self.eps_ts = eps_ts

#     def get_FK(self, node: Node2):
#         tcp = []
#         for robot_name in self.robots:
#             robot = self.kingraph.objects[robot_name]
#             q = node.q[robot_name]
#             node.T[robot_name] = robot.forward_kinematics(q)
#             tcp.append(node.T[robot_name].trans)
        
#         if len(tcp) == 2:
#             node.tcp_dist = np.linalg.norm(tcp[0] - tcp[1])

#     def plan(
#         self,
#         config_init: Config,
#         kingraph: KinGraph,
#         holding_robot_name: str, #holding
#         handover_robot_name: str,
#         fix_grasp = None,
#         grasp_sampler = None,
#     ):
#         self.kingraph = kingraph
#         self.robots : Dict[str, Panda]= {name:self.kingraph.objects[name] for name in self.kingraph.robots}
#         self.config_init = config_init
#         self.root = Node2(self.config_init.q)
#         self.get_FK(self.root)
#         tree = Tree2(self.root)
#         obj_name = kingraph.relation.child[holding_robot_name]
#         grasp_holding: Grasp = kingraph.kin_edge[obj_name]
#         self.fix_grasp = fix_grasp
#         self.grasp_sampler = grasp_sampler
        
#         for _ in range(self.max_iter):
#             self.extend_randomly(tree)
#             if np.random.random() < 0.5:
#                 result = self.extend_to_goal(tree, grasp_holding, handover_robot_name)
#                 if result is not None:
#                     return result #grasp_handover, traj # result #obj_pose_mid #Config(last_node.q)  # TODO: Prune
#         return None
    
#     def get_random_node(self) -> Node2:
#         q = {}
#         for robot_name in self.robots:
#             value = self.robots[robot_name].get_random_arm_angles()
#             q[robot_name] = value
#         return Node2(q)

#     def limit_step_size(self, q_delta: np.ndarray, q_delta_max: Optional[np.ndarray]=None):
#         if q_delta_max is None:
#             q_delta_max = self.q_delta_max
#         mag = np.linalg.norm(q_delta, np.inf)
#         if mag > q_delta_max:
#             q_delta = q_delta / mag * q_delta_max
#         return q_delta

#     def move_cspace(self, node_rand: Node2, node_tree: Node2, robot_name: str):
#         q_err = node_rand.q[robot_name] - node_tree.q[robot_name]
#         q_delta = self.limit_step_size(q_err, self.q_delta_max)
#         q_new = node_tree.q[robot_name] + q_delta
#         node_new = node_tree.copy()
#         node_new.q[robot_name] = q_new
#         if self.kingraph.is_collision(Config(node_new.q)):
#             return None
#         self.get_FK(node_new)
#         return node_new

#     def extend_randomly(self, tree: Tree2):
#         node_rand = self.get_random_node()
#         node_tree = tree.nearest_cs(node_rand)
        
#         node_old = node_tree.copy()
#         robot_order = list(node_tree.q.keys())
#         np.random.shuffle(robot_order)
#         for robot_name in robot_order:
#             node_new = self.move_cspace(node_rand, node_old, robot_name)
#             if node_new is None:
#                 node_new = node_old
#                 break
#             node_old = node_new
#         if node_tree == node_new:
#             return
#         tree.add_node(node_new, node_tree)
#         return 
    
#     def extend_to_goal(self, tree: Tree2, grasp_holding: Grasp, handover_robot_name: str):
#         movable_name = grasp_holding.obj_name
#         if self.fix_grasp is None:
#             grasp_handover: Grasp = self.grasp_sampler.sample_grasp_bunch(movable_name, handover_robot_name, self.kingraph, cond_grasp=grasp_holding, k=5)
#         else:
#             grasp_handover: Grasp = self.fix_grasp
#         #grasp_rand = self.kingraph.sample_grasp(movable_name, handover_robot_name)
#         node_tree = tree.nearest_tcp_dist()

#         robot_order = list(node_tree.q.keys())
#         np.random.shuffle(robot_order)
#         node_old = node_tree
        
#         nodes = []
#         for _ in range(100):
#             #pose target
#             tcp_holding = node_old.T[grasp_holding.parent_name]
#             tcp_handover = node_old.T[handover_robot_name]
#             obj_pose_holding = tcp_holding * grasp_holding.tf.inverse()
#             obj_pose_handover = tcp_handover * grasp_handover.get_pre_pose(grasp_handover.tf).inverse()
#             pos_mid = (obj_pose_holding.trans + obj_pose_handover.trans)/2
#             orn_mid = slerp(obj_pose_holding.rot.as_quat(), obj_pose_handover.rot.as_quat(), 0.5)
#             obj_pose_mid = Pose(Rotation.from_quat(orn_mid), pos_mid)
#             for robot_name in robot_order:
#                 if robot_name == handover_robot_name:
#                     grasp_target = obj_pose_mid * grasp_handover.tf
#                     ee_target = grasp_handover.get_pre_pose(grasp_target)
#                 else:
#                     ee_target = obj_pose_mid * grasp_holding.tf
#                 node_new = self.move_tspace(node_old, ee_target, robot_name)
            
#                 if node_new is None:
#                     node_tree.excluded = True
#                     return None
                
#                 node_old = node_new
#             nodes.append(node_new)
#             if distance_ts(node_new.T[robot_name], ee_target) < 0.01:
#                 grasp_holding.set_obj_handover_pose(obj_pose_mid)
#                 grasp_handover.set_obj_handover_pose(obj_pose_mid)
#                 return grasp_handover, self.solution(tree, nodes, node_tree)
#                 # #self.solution(tree, nodes, node_tree)
    
#     def solution(self, tree: Tree2, nodes: Node2, node_tree: Node2):
#         path = []
#         path += tree.backtrack(node_tree)
#         path += nodes
#         config_path = []
#         for node in path:
#             config_path.append(Config(node.q))
#         return config_path

#     def move_tspace(self, node_tree: Node2, pose_target: Pose, robot_name: str):
#         damping = 0.1
#         q_curr: np.ndarray = node_tree.q[robot_name]
#         pose_curr: Pose = node_tree.T[robot_name]

#         pos_err = pose_target.trans - pose_curr.trans
#         orn_err = orn_error(pose_target.rot.as_quat(), pose_curr.rot.as_quat())
#         err = np.hstack([pos_err, orn_err*2])

#         jac = self.robots[robot_name].get_jacobian(q_curr)
#         lmbda = np.eye(6) * damping ** 2
#         jac_pinv = jac.T @ np.linalg.inv(jac @ jac.T + lmbda)
#         q_delta = self.limit_step_size(jac_pinv @ err)
#         q_new = node_tree.q[robot_name] + q_delta
#         node_new = node_tree.copy()
#         node_new.q[robot_name] = q_new
#         if self.kingraph.is_collision(Config(node_new.q)):
#             return None
#         self.get_FK(node_new)
#         return node_new


# class DualArmJinvRRT:
#     def __init__(
#         self,
#         eps: float = 0.1,
#         p_global_explore: float = 0.5,
#         p_constraint_explore: float = 0.5,
#         max_iter: int = 100,
#         q_delta_max: float = 0.2,
#         DLS_damping: float = 0.1,
#         eps_ts: float = 0.05
#     ):
#         self.eps = eps
#         self.p_global_explore = p_global_explore
#         self.p_constraint_explore = p_constraint_explore
#         self.max_iter = max_iter
#         self.q_delta_max = q_delta_max
#         self.DLS_damping = DLS_damping
#         self.eps_ts = eps_ts

#     def plan(
#         self,
#         config: Config,
#         kingraph: KinGraph,
#         grasp1: Grasp, #holding
#         grasp2: Grasp,
#         action: Action
#     ):
#         self.kingraph = kingraph
#         self.config = config
#         self.robot_r: Panda = self.kingraph.objects["robot1"]
#         self.robot_l: Panda = self.kingraph.objects["robot2"]
#         self.holding_robot = self.kingraph.relation.grasp[grasp1.obj_name]
#         self.move_robot = "robot1" if self.holding_robot == "robot2" else "robot2"
#         q_r, q_l = config.q["robot1"], config.q["robot2"]
#         T_r, T_l = self.robot_r.forward_kinematics(q_r), self.robot_l.forward_kinematics(q_l)
#         node = Node2(q_l=q_l, q_r=q_r, T_l=T_l, T_r=T_r)
#         tree = Tree2(node)

#         #check next mode feasibility
#         for _ in range(self.max_iter):
#             p1 = np.random.random()
#             if p1 < self.p_global_explore:
#                 self.extend_randomly(tree)
#             else:
#                 # config_final = self.extend_to_pose(tree, grasp1, grasp2)
#                 p2 = np.random.random()
#                 if p2 < self.p_constraint_explore:
#                     movable: Movable = self.kingraph.objects[grasp1.obj_name]
#                     grasp_target = movable.sample_grasp()
#                 else:
#                     grasp_target = grasp2
                
#                 # if T_target is None:
#                 #     continue
#                 #kingraph.sm.view_frame(T_target, "target")
#                 node_final = self.extend_to_constraint(tree, grasp1, grasp_target)
            
#                 if node_final:
#                     return grasp_target, self.backtrack(tree, node_final)
#         return None, None

#     def backtrack(self, tree: Tree, last_node: Node) -> List[Config]:
#         traj_ = tree.backtrack(last_node)
#         traj = []
#         for node in traj_:
#             config = self.config.copy()
#             config.q["robot2"] = node.q_l
#             config.q["robot1"] = node.q_r
#             traj.append(config)
#         return traj

#     def is_collision(self, node: Node2):
#         q = {"robot1":node.q_r, "robot2":node.q_l}
#         config = Config(q=q)
#         return self.kingraph.is_collision(config)

#     def extend_randomly(self, tree: Tree2):
#         q_target_l = self.robot_l.get_random_arm_angles()
#         q_target_r = self.robot_r.get_random_arm_angles()
#         node_tree = tree.nearest_cs(q_target_l, q_target_r)
#         q_l, q_r = node_tree.q_l, node_tree.q_r

#         is_left_advanced = is_right_advanced = True
#         node_new = deepcopy(node_tree)
#         node_new.q_l = self.steer_cs(q_target_l, node_tree.q_l)
        
#         if self.is_collision(node_new):
#             node_new.q_l = q_l
#             is_left_advanced = False
#         node_new.q_r = self.steer_cs(q_target_r, node_tree.q_r)
#         if self.is_collision(node_new):
#             node_new.q_r = q_r
#             is_right_advanced = False
        
#         if (is_left_advanced == False) & (is_right_advanced == False):
#             return None
        
#         node_new.T_l = self.robot_l.forward_kinematics(node_new.q_l)
#         node_new.T_r = self.robot_r.forward_kinematics(node_new.q_r)
#         tree.add_node(node_new, node_tree)

#     def steer_cs(self, q_target: np.ndarray, q: np.ndarray):
#         mag = np.linalg.norm(q_target - q)
#         if mag < 0.2:
#             q_delta = q_target - q
#         else:
#             q_delta = (q_target - q) / mag * 0.2
#         return q_delta + q
    
#     def steer(
#         self,
#         q: np.ndarray,
#         jac: np.ndarray,
#         curr_pose: Pose,
#         target_pose: Pose,
#         q_delta_max: Optional[float] = None
#     ) -> np.ndarray:
#         if q_delta_max is None:
#             q_delta_max = self.q_delta_max
#         pos_err = target_pose.trans - curr_pose.trans
#         orn_err = orn_error(target_pose.rot.as_quat(), curr_pose.rot.as_quat())
#         err = np.hstack([pos_err, orn_err*2])
#         lmbda = np.eye(6) * self.DLS_damping ** 2
#         jac_pinv = jac.T @ np.linalg.inv(jac @ jac.T + lmbda)
#         q_delta = self.limit_step_size(jac_pinv @ err, q_delta_max)
#         return q_delta + q

#     def extend_to_constraint(
#         self,
#         tree: Tree2,
#         grasp1: Grasp,
#         grasp2: Grasp
#     ) -> Optional[Config]:
#         EXTEND_IN_A_ROW = 5
#         if self.kingraph.relation.grasp[grasp1.obj_name] == "robot1":
#             # grasping right(robot1) grasp1 = robot1 = grasp_r
#             grasp_r, grasp_l = grasp1.tf, grasp2.tf
#             constraint_l = grasp1.get_pre_pose(grasp_l)
#             constraint_r = grasp_r
#         else:
#             # grasping left(robot2) grasp2 = robot1 = grasp_r
#             grasp_l, grasp_r = grasp1.tf, grasp2.tf
#             constraint_l = grasp_l
#             constraint_r = grasp2.get_pre_pose(grasp_r)
        
#         Tconstraint_l_r = constraint_l.inverse() * constraint_r
#         node_tree = tree.nearest_constraint(Tconstraint_l_r)

#         node_old = node_tree
#         nodes = []
#         is_goal = False
#         for _ in range(EXTEND_IN_A_ROW):
#             Tobj_l = node_old.T_l * grasp_l.inverse()
#             Tobj_r = node_old.T_r * grasp_r.inverse()
#             pos_mid = Tobj_l.trans + (Tobj_r.trans - Tobj_l.trans)/2
#             orn_mid = slerp(Tobj_l.rot.as_quat(), Tobj_r.rot.as_quat(), 0.5)
#             obj_mid = Pose(Rotation.from_quat(orn_mid), pos_mid)

#             T_target_l = obj_mid * constraint_l
#             T_target_r = obj_mid * constraint_r
            
#             node_new = self.move_arm(node_old, T_target_l, T_target_r)
            
#             if node_new is None:
#                 return None

#             node_new.T_l = self.robot_l.forward_kinematics(node_new.q_l)
#             node_new.T_r = self.robot_r.forward_kinematics(node_new.q_r)
#             nodes.append(node_new)

#             if distance_ts(node_new.T_l*Tconstraint_l_r, node_new.T_r) < 0.01:
#                 #final checking
#                 q = {"robot1":node_new.q_r, "robot2":node_new.q_l}
#                 config = Config(q)
#                 if self.move_robot == "robot1":
#                     T_target = Tobj_l * grasp_r    
#                 else:
#                     T_target = Tobj_r * grasp_l
#                 result = self.check_mode_switch(self.move_robot, config, T_target)
                    
#                 if result:
#                     is_goal = True
#                     break
#             node_old = node_new

#         node_prev = node_tree
#         for node in nodes:
#             tree.add_node(node, node_prev)
#             node_prev = node
#         return node_new if is_goal else None

#     def check_mode_switch(
#         self, 
#         robot_name: str,
#         node_final: Config, 
#         T_target: Pose
#     ):
#         EPS = 0.01
#         result = False
#         traj = []
#         robot: Panda = self.kingraph.objects[robot_name]
#         with robot.no_set_joint():
#             config = node_final.copy()
#             for _ in range(10):
#                 q_old = config.q[robot_name]
#                 T = robot.forward_kinematics(q_old)
#                 q = self.steer(
#                     q=q_old,
#                     jac=robot.get_jacobian(q_old),
#                     curr_pose=T,
#                     target_pose=T_target,
#                     q_delta_max=0.05
#                 )
#                 config_new = node_final.copy()
#                 config_new.q[robot_name] = q
#                 robot.set_joint_angles(q)
#                 T = robot.get_ee_pose()
#                 self.kingraph.assign()
#                 if not self.kingraph.is_collision(config_new):
#                     traj.append(config_new)
#                     if distance_ts(T, T_target) < EPS:
#                         result = traj
#                         break
#                     config = config_new
#                 else:
#                     break
                
#         self.kingraph.assign()
#         return result
    
#     def move_arm(
#         self,
#         node_old:Node2,
#         T_target_l: Pose,
#         T_target_r: Pose
#     ) -> Node2:
#         is_left_advanced = is_right_advanced = True
#         node_new = deepcopy(node_old)
#         robot_r = self.kingraph.objects["robot1"]
#         robot_l = self.kingraph.objects["robot2"]

#         node_new.q_l = self.steer_ts(
#             node_old.q_l, T_target_l, node_old.T_l,
#             robot_l
#         )
#         if self.is_collision(node_new):
#             node_new.q_l = node_old.q_l
#             is_left_advanced = False

#         node_new.q_r = self.steer_ts(
#             node_old.q_r, T_target_r, node_old.T_r, 
#             robot_r
#         )
#         if self.is_collision(node_new):
#             node_new.q_r = node_old.q_r
#             is_right_advanced = False
        
#         if (is_left_advanced == False) & (is_right_advanced == False):
#             return None
#         return node_new
    
#     def steer_ts(
#         self, 
#         q: np.ndarray,
#         T_target: Pose,  
#         T: Pose, 
#         robot: Panda
#     ):
#         damping = 0.1
#         pos_err = T_target.trans - T.trans
#         orn_err = orn_error(T_target.rot.as_quat(), T.rot.as_quat())
#         err = np.hstack([pos_err, orn_err*1.5])
#         jac = robot.get_jacobian(q)
#         lmbda = np.eye(6) * damping ** 2
#         jac_pinv = jac.T @ np.linalg.inv(jac @ jac.T + lmbda)
#         q_delta = self.limit_step_size(jac_pinv @ err)
#         return q_delta + q
    
#     def limit_step_size(self, q_delta: np.ndarray, q_delta_max: Optional[np.ndarray]=None):
#         if q_delta_max is None:
#             q_delta_max = self.q_delta_max
#         mag = np.linalg.norm(q_delta, np.inf)
#         if mag > q_delta_max:
#             q_delta = q_delta / mag * q_delta_max
#         return q_delta


# class RJRRT:
#     def __init__(
#         self,
#         eps: float = 0.1,
#         p_global_explore: float = 0.5,
#         p_constraint_explore: float = 0.5,
#         max_iter: int = 100,
#         q_delta_max: float = 0.2,
#         DLS_damping: float = 0.1,
#         eps_ts: float = 0.05
#     ):
#         self.eps = eps
#         self.p_global_explore = p_global_explore
#         self.p_constraint_explore = p_constraint_explore
#         self.max_iter = max_iter
#         self.q_delta_max = q_delta_max
#         self.DLS_damping = DLS_damping
#         self.eps_ts = eps_ts

    
#     def grow_tree(
#         self,
#         config: Config,
#         #tree: Tree,
#         kingraph: KinGraph,
#         grow_target: KinEdge,
#         action: Action
#     ):
#         self.kingraph = kingraph
#         tree = Tree(config)

#         #check next mode feasibility
#         for _ in range(self.max_iter):
#             p1 = np.random.random()
#             if p1 < self.p_global_explore:
#                 self.extend_randomly(tree)
#             else:

#                 # p2 = np.random.random()
#                 # if p2 < self.p_constraint_explore:
#                 #     edge, T_target = self.get_edge_Ttarget(action, grow_target, is_random=True)
#                 # else:
#                 #     edge, T_target = self.get_edge_Ttarget(action, grow_target, is_random=False)
                
#                 # if T_target is None:
#                 #     continue
#                 #kingraph.sm.view_frame(T_target, "target")
#                 node_final = self.extend_to_pose(T_target, tree)
            
#                 if node_final:
#                     return edge, node_final
#         return None, None

#     def get_edge_Ttarget(self, action: Action, grow_target: List[KinEdge], is_random=False):
#         movable: Movable = self.kingraph.objects[action.obj_name]

#         if (not is_random) & (len(grow_target) != 0):
#             edge = np.random.choice(grow_target)
#             if action.name == "pick":
#                 grasp = edge
#                 placement = self.kingraph.kin_edge[action.obj_name]
#             elif action.name == "place":
#                 placement = edge
#                 grasp = self.kingraph.kin_edge[movable.name]
            
#             placeable: Fixed = self.kingraph.objects[placement.placeable_name]
#             obj_pose = placeable.get_base_pose() * placement.tf.inverse()
#             T_target = obj_pose * grasp.tf
#             self.kingraph.sm.view_frame(T_target, "target")
            
#             # if self.kingraph.tool_collision_check(grasp, placement):
#             #     grow_target.remove(edge)
#             #     return None, None
#         else:
#             if action.name == "pick":
#                 grasp = movable.sample_grasp()
#                 placement = self.kingraph.kin_edge[action.obj_name]
#                 edge = grasp

#             elif action.name == "place":                
#                 placement = self.kingraph.sample_placement(movable.name, action.placeable_name)
#                 grasp = self.kingraph.kin_edge[movable.name]
#                 edge = placement

#             if self.kingraph.tool_collision_check(grasp, placement):
#                 return None, None
        
        
#         placeable: Fixed = self.kingraph.objects[placement.placeable_name]
#         obj_pose = placeable.get_base_pose() * placement.tf.inverse()
#         T_target = obj_pose * grasp.tf
        
#         if action.name == "pick":
#             T_target = grasp.get_pre_pose(T_target)
#         elif action.name == "placement":
#             T_target = placement.get_pre_pose(T_target)
        
#         return edge, T_target

#     def extend_to_pose(self, T_target: Pose, tree: Tree):
#         def add_nodes(nodes:Config, parent:Config, tree:Tree):
#             for node in nodes:
#                 tree.add_node(node, parent)
#                 parent = node

#         EXTEND_IN_A_ROW = 5
#         node_tree = tree.nearest_tspace(T_target)

#         node_old = node_tree
#         nodes = []
#         for _ in range(EXTEND_IN_A_ROW):
#             node_new = node_old.copy()
#             node_new.q = self.steer(
#                 q=node_old.q, 
#                 jac=self.kingraph.robot.get_jacobian(node_old.q), 
#                 curr_pose=node_old.T, 
#                 target_pose=T_target
#             )

#             if not self.kingraph.is_collision(node_new):
#                 node_new.T = self.kingraph.robot.forward_kinematics(node_new.q)
#                 nodes.append(node_new)

#                 if distance_ts(T_target, node_new.T) < self.eps_ts:
#                     # goal(reached)
#                     node_final = node_new.copy()
#                     node_final.q = self.steer(
#                         q=node_new.q, 
#                         jac=self.kingraph.robot.get_jacobian(node_new.q), 
#                         curr_pose=node_new.T, 
#                         target_pose=T_target
#                     )
#                     nodes.append(node_final)
#                     add_nodes(nodes, node_tree, tree)
#                     return node_final
                
#                 node_old = node_new
#             else:
#                 # if collision, reject all extension
#                 break

#         # advanced
#         add_nodes(nodes, node_tree, tree)
#         return None

#     def extend_randomly(self, tree: Tree):
#         q_target = Config(self.kingraph.robot.get_random_arm_angles())
#         node_tree = tree.nearest(q_target)
#         q_delta = self.limit_step_size(q_target.q - node_tree.q)
#         node_new = node_tree.copy()
#         node_new.q = node_tree.q + q_delta
#         if not self.kingraph.is_collision(node_new):
#             node_new.T = self.kingraph.robot.forward_kinematics(node_new.q)
#             tree.add_node(node_new, node_tree)
    
#     def limit_step_size(self, q_delta: np.ndarray, q_delta_max: Optional[np.ndarray]=None):
#         if q_delta_max is None:
#             q_delta_max = self.q_delta_max
#         mag = np.linalg.norm(q_delta, np.inf)
#         if mag > q_delta_max:
#             q_delta = q_delta / mag * q_delta_max
#         return q_delta
    
#     def steer(
#         self,
#         q: np.ndarray,
#         jac: np.ndarray,
#         curr_pose: Pose,
#         target_pose: Pose,
#         q_delta_max: Optional[float] = None
#     ) -> np.ndarray:
#         if q_delta_max is None:
#             q_delta_max = self.q_delta_max
#         pos_err = target_pose.trans - curr_pose.trans
#         orn_err = orn_error(target_pose.rot.as_quat(), curr_pose.rot.as_quat())
#         err = np.hstack([pos_err, orn_err*2])
#         lmbda = np.eye(6) * self.DLS_damping ** 2
#         jac_pinv = jac.T @ np.linalg.inv(jac @ jac.T + lmbda)
#         q_delta = self.limit_step_size(jac_pinv @ err, q_delta_max)
#         return q_delta + q

# class ModeForest:
#     def __init__(
#         self,
#         eps: float = 0.1,
#         p_goal: float = 0.4,
#         max_iter: int = 100,
#         q_delta_max: float = 0.1,
#         DLS_damping: float = 0.1,
#     ):
#         self.eps = eps
#         self.p_goal = p_goal
#         self.max_iter = max_iter
#         self.q_delta_max = q_delta_max
#         self.DLS_damping = DLS_damping
    
#     def distance(self, node1:Config, node2:Config):
#         return np.linalg.norm(node1.q - node2.q)
    
#     def grow_tree(
#         self,
#         goal_config: Config,
#         tree: Tree,
#         kingraph: KinGraph,
#     ):
#         self.kingraph = kingraph
        
#         for _ in range(self.max_iter):
#             p = np.random.random()
#             if p < self.p_goal:
#                 node_rand = goal_config.copy()
#             else:
#                 node_rand = Config(kingraph.robot.get_random_arm_angles())
#             if self.extend(tree, node_rand) == "reached":
#                 return self.last_node
#         return None
    
#     def grow_bi_tree(self, tree_start: Config, tree_goal: Tree, kingraph: KinGraph):
#         self.kingraph = kingraph
#         tree_a = tree_start
#         tree_b = tree_goal
#         for i in range(self.max_iter):
#             node_rand = Config(kingraph.robot.get_random_arm_angles())
            
#             if not self.extend(tree_a, node_rand) == "trapped":
#                 if self.connect(tree_b, self._node_new) == "reached":
#                     return self.get_path(tree_start, tree_goal)
#             (tree_a, tree_b) = (tree_b, tree_a)
#         return []
    
#     def extend(self, tree: Tree, node_rand: Config):
#         node_near = tree.nearest(node_rand)
#         node_new = self.control(node_near, node_rand)
#         if node_new is not None:
#             tree.add_node(node_new, node_near)
#             if self.distance(node_rand, node_new) < self.eps:
#                 self.last_node = node_new
#                 return "reached"
#             else:
#                 self._node_new = node_new #save this to "connect"
#                 return "advanced"
#         return "trapped"
    
#     def connect(self, tree, node):
#         result = "advanced"
#         while result == "advanced":
#             result = self.extend(tree, node)
#         return result

#     def control(self, node_near:Config, node_rand:Config):
#         mag = self.distance(node_near, node_rand)
#         if mag <= self.eps:
#             node_new = node_rand.copy()
#             node_new.index = -1
#         else:
#             q_err = node_rand.q - node_near.q
#             q_delta = self.limit_step_size(q_err, self.q_delta_max)
#             q_new = node_near.q + q_delta
#             node_new = Config(q_new)

#         if not self.kingraph.is_collision(node_new):
#             return node_new
#         else:
#             return None
    
#     def get_path(self, tree_start: Tree, tree_goal: Tree):
#         node_tree_start = tree_start.nearest(self.last_node)
#         node_tree_goal = tree_goal.nearest(self.last_node)
#         path_from_start = tree_start.backtrack(node_tree_start)
#         path_from_goal = tree_goal.backtrack(node_tree_goal)
        
#         return [*path_from_start, *path_from_goal[::-1]]

#     def limit_step_size(self, q_delta: np.ndarray, q_delta_max: Optional[np.ndarray]=None):
#         if q_delta_max is None:
#             q_delta_max = self.q_delta_max
#         mag = np.linalg.norm(q_delta, np.inf)
#         if mag > q_delta_max:
#             q_delta = q_delta / mag * q_delta_max
#         return q_delta
    
    # def check_mode_switch(
    #     self, 
    #     node_final: Config, 
    #     T_target: Transform,
    #     kingraph: KinGraphReal
    # ):
    #     EPS = 0.01
    #     result = False
    #     traj = []
    #     with kingraph.robot.no_set_joint():
    #         config = node_final.copy()
    #         for _ in range(10):
    #             q = self.steer(
    #                 q=config.q,
    #                 jac=kingraph.robot.get_jacobian(config.q),
    #                 curr_pose=kingraph.robot.forward_kinematics(config.q),
    #                 target_pose=T_target,
    #                 q_delta_max=0.05
    #             )
    #             config_new = Config(q)
    #             kingraph.robot.set_arm_angles(config_new.q)
    #             config.T = kingraph.robot.get_ee_pose()
    #             kingraph.assign()
    #             if not kingraph.is_collision(config):
    #                 traj.append(config_new)
    #                 if distance_ts(config.T, T_target) < EPS:
    #                     result = traj
    #                     break
    #                 config = config_new
    #             else:
    #                 break
                
    #     kingraph.assign()
    #     return result
    
    # def steer(
    #     self,
    #     q: np.ndarray,
    #     jac: np.ndarray,
    #     curr_pose: Transform,
    #     target_pose: Transform,
    #     q_delta_max: Optional[float] = None
    # ) -> np.ndarray:
    #     if q_delta_max is None:
    #         q_delta_max = self.q_delta_max
    #     pos_err = target_pose.translation - curr_pose.translation
    #     orn_err = orn_error(target_pose.rotation.as_quat(), curr_pose.rotation.as_quat())
    #     err = np.hstack([pos_err, orn_err*2])
    #     lmbda = np.eye(6) * self.DLS_damping ** 2
    #     jac_pinv = jac.T @ np.linalg.inv(jac @ jac.T + lmbda)
    #     q_delta = self.limit_step_size(jac_pinv @ err, q_delta_max)
    #     return q_delta + q