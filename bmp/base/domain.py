from pybullet_suite import *
from ..utils.utils import *
from copy import copy, deepcopy
from dataclasses import dataclass, field
from itertools import product
from .elements import *
from .checker import Checker

class TAMPDomain:
    def __init__(self, gui):
        self.movables: Dict[str, Movable] = {}
        self.regions: Dict[str, Region] = {}
        self.robots: Dict[str, Robot] = {}
        self.envs: Dict[str, Body] = {}

        self.world = BulletWorld(gui=gui)
        self.checker = Checker(not gui)
        self.saved_state = None
        movables, regions, robots, envs = self.set_task_scene()
        self.save()
        self.set_tamp_objects(movables, regions, robots)
        self.set_init_mode_config()
        self.envs = envs
        self.domain_pddl = None

    @property
    def objects(self):
        return {**self.movables, **self.regions, **self.robots}

    @property
    def fixed(self):
        return {**self.envs, **self.regions}
    
    @property
    def all_instances(self):
        return {**self.objects, **self.envs}

    def set_task_scene(self):
        # this should save self.init_state
        # this should output movables, regions, robots
        pass 
    
    def set_tamp_objects(self, movables: Dict, regions: Dict, robots: Dict):
        self.movables = None
        self.regions = None
        self.robots = None
        pass
    
    def set_init_mode_config(self):
        # set initial mode of the problem
        self.init_mode = None
        pass

    def save(self):
        self.saved_state = self.world.save_state()
    
    def reset(self):
        self.world.restore_state(self.saved_state)
    
    def set_config(self, config: Config):
        for robot in config.q.keys():
            q = config.q[robot]
            self.robots[robot].set_joint_angles(q)
    

    # TODO:
    def assign(self, mode:Mode, config:Optional[Config]=None):
        def assign_obj(
            obj: str, 
            parents:Dict[str, str], 
            atts: Dict[str, Attachment],
            assigned: List[str]
        ):
            parent_name = parents[obj]
            parent_type = type(self.objects[parent_name])
            if parent_type is Panda:
                parent_pose = self.robots[parent_name].get_ee_pose()
            elif parent_type is Region:
                parent_pose = self.regions[parent_name].get_base_pose()
            elif parent_type is Movable:
                parent_pose = assign_obj(parent_name, parents, atts, assigned)
            
            obj_pose = parent_pose * atts[obj].tf.inverse()
            self.movables[obj].set_base_pose(obj_pose)
            assigned.append(obj)
            return obj_pose
        
        if config:
            self.set_config(config)
        
        # make graph
        parents = {}
        for obj, att in mode.attachments.items():
            parents[obj] = att.parent_name
        
        assigned = []
        for obj, att in mode.attachments.items():
            if obj not in assigned:
                assign_obj(obj, parents, mode.attachments, assigned)
        
        for obj, att in mode.attachments.items():
            if isinstance(att, Grasp):
                robot: Panda = self.robots[att.parent_name]
                robot.open(att.width)
    
    def is_collision(self, mode:Mode, config: Config):
        self.assign(mode, config)

        #self collision
        for robot in self.robots:
            if self.world.is_self_collision(robot):
                return True
        
        #collision between robots
        for robot1, robot2 in combinations(self.robots, 2):
            if self.world.is_body_pairwise_collision(
                body=robot1, obstacles=[robot2]
            ):
                return True
        
        # robot-fixed
        for robot, fixed in product(self.robots, self.fixed):
            if self.world.is_body_pairwise_collision(
                body=robot, obstacles=[fixed]
            ):
                return True

        #movable collision
        for movable in self.movables:
            obstacles = list(self.all_instances.keys())
            obstacles.remove(movable) #check except myself
            if movable in mode.attachments.keys():
                parent = mode.attachments[movable].parent_name
                obstacles.remove(parent) #check except parent
                    
            if self.world.is_body_pairwise_collision(
                body=movable, obstacles=obstacles):
                return True
        return False
    
    def sample_grasp(self, obj_name: str, parent_name: str):
        movable: Movable = self.objects[obj_name]
        grasp = movable.sample_grasp()
        grasp.parent_name = parent_name
        return grasp

    def sample_placement(self, obj_name: str, parent_name: str, yaw=None):
        movable: Movable = self.objects[obj_name]
        placeable: TAMPObject = self.objects[parent_name]
        sssp = placeable.sssp
        point = np.random.uniform(sssp.lower, sssp.upper)
        point = placeable.get_base_pose().transform_point(point)
        sop = movable.sample_sop()
        placement = Placement.from_point_and_sop(
            obj_name, 
            parent_name, 
            placeable.get_base_pose(),
            point,
            sop,
            yaw=yaw
        )
        return placement
    
    def get_curr_placement(self, obj_name: str, parent_name: str, sop:SOP, yaw=0):
        movable: Movable = self.objects[obj_name]
        placeable: TAMPObject = self.objects[parent_name]
        
        point = movable.get_base_pose().trans
        z = placeable.get_base_pose().trans[-1] + placeable.sssp.upper[-1]
        point[-1] = z
        placement = Placement.from_point_and_sop(
            obj_name, 
            parent_name, 
            placeable.get_base_pose(),
            point,
            sop,
            yaw=yaw
        )
        return placement

    def sample_instance(self, action_name:str, obj_name:str, parent_name:str):
        if action_name == "pick":
            instance = self.sample_grasp(obj_name, parent_name)
        elif action_name == "place":
            instance = self.sample_placement(obj_name, parent_name)
        return instance