from pathlib import Path

from pybullet_suite import *
from bmp.utils.utils import *
from bmp.base import *
import os
import shutil

##############################
class DomainKitchen(TAMPDomain):
    def __init__(self, gui):
        super().__init__(gui)
        self.domain_pddl = Path(__file__).parent / Path("pddl/domain.pddl")
        self.domain_name = "KITCHEN"
        with open(self.domain_pddl, "r") as f:
            self.domain_pddl_string = f.read()
        
        #types
        self.action_info = {
            "geometric":["pick", "place"],
            "non-geometric":["wash", "cook"]
        }
        self.region_types = ["sink", "oven", "dish"]
        self.movable_types = ["food"]
        self.predicates = ["on", "handempty", "holding", "clear", "cleaned", "coocked"]

    def set_task_scene(self):
        with no_output():
            with self.world.no_rendering():
                sm = BulletSceneMaker(self.world)
                plane = sm.create_plane(z_offset=-0.4)
                ground = sm.create_table("ground", 2, 2, 0.4) #ground
                hand = Gripper(self.world)

                # set environment
                envs = {}
                envs["table"] = sm.create_table(
                    "table", 0.4, 1.2, 0.2, 
                    x_offset=0.4, y_offset=0, z_offset=0.2)
                
                # set regions
                regions = {}
                names = ["dish1", "sink1", "oven1"]
                w, h = 0.22, 0.02
                half_extents = [w/2, w/2, h/2]
                positions = [
                    [0.4, 0, 0.2+0.01],
                    [0.4, +0.3, 0.2+0.01],
                    [0.4, -0.3, 0.2+0.01]
                ]
                colors = [
                    [0.6, 0.6, 0.6, 1],
                    [0, 0, 1, 1],
                    [1, 0, 0, 1]
                ]
                for idx, name in enumerate(names):
                    regions[name] = sm.create_box(
                        body_name=name, 
                        half_extents=half_extents, 
                        mass=1., 
                        position=positions[idx],
                        rgba_color=colors[idx])
                
                # set movables
                movables = {}
                w, h = 0.05, 0.07
                half_extents = [w/2, w/2, h/2]
                gap = w+0.02
                names = ["box1", "box2", "box3", "box4"]
                positions = [
                    [0.4, 0, 0.2+0.02+0.05],
                    [0.4, -gap, 0.2+0.02+0.05],
                    [0.4, +gap, 0.2+0.02+0.05],
                    [0.4-gap, 0, 0.2+0.02+0.05]
                ]
                colors = [
                    [1, 0, 0, 1],
                    [0, 1, 0, 1],
                    [0, 0, 1, 1],
                    [0.7, 0.7, 0, 1],
                ]
                for idx, name in enumerate(names):
                    movables[name] = sm.create_box(
                        body_name=name, 
                        half_extents=half_extents, 
                        mass=1., 
                        position=positions[idx],
                        rgba_color=colors[idx])

                # set robot
                robots = {"robot":self.world.load_robot("robot", robot_class=Panda)}
        
        return movables, regions, robots, envs

    def set_tamp_objects(self, movables:Dict, regions:Dict, robots:Dict):
        def get_top_grasps(body: Body):
            max_grasp_width = 0.15
            grasp_depth = 0.015
            center, (w, l, h) = body.get_AABB_wrt_obj_frame()
            rot = Rotation.from_euler("xyz",[np.pi,0,0])
            trans = [0,0,h/2-grasp_depth]
            grasp_poses = []
            grasp_widths = []
            if w <= max_grasp_width:
                grasp_poses += [
                    Pose(rot=rot, trans=trans),
                    Pose(rot=Rotation.from_euler("xyz",[0,0,np.pi])*rot,trans=trans)
                ]
                grasp_widths += [w, w]
            if l <= max_grasp_width:
                grasp_poses += [
                    Pose(rot=Rotation.from_euler("xyz",[0,0,np.pi/2])*rot,trans=trans),
                    Pose(rot=Rotation.from_euler("xyz",[0,0,np.pi*3/2])*rot,trans=trans)
                ]
                grasp_widths += [l, l]
            grasps = [Grasp(name, tf=tf, width=width) 
                for tf, width in zip(grasp_poses, grasp_widths)]
            return grasps
        def get_sops(body: Body):
            center, (w, l, h) = body.get_AABB_wrt_obj_frame()
            sop = SOP(Rotation.identity(), np.array([0,0,1]), h/2)
            sops = [sop]
            return sops
        def get_sssp(body: Body):
            lower, upper = body.get_AABB_wrt_obj_frame(output_center_extent=False)
            lower[-1] = upper[-1]
            return SSSP(lower, upper)

        self.movables = {}
        for name, body in movables.items():
            grasps = get_top_grasps(body)
            sops = get_sops(body)
            sssp = get_sssp(body)
            self.movables[name] = Movable.from_body(body, name, grasps, sops, sssp)
        
        self.regions = {}
        for name, body in regions.items():
            sssp = get_sssp(body)
            self.regions[name] = Region.from_body(body, name, sssp)
        
        self.robots = {}
        for name, robot in robots.items():
            self.robots[name] = robot

    def set_init_mode_config(self):
        #all movables are placed to the dish
        att_list = []
        for movable_name in self.movables.keys():
            movable = self.movables[movable_name]
            sop = movable.sops[0]
            placement = self.get_curr_placement(movable_name, "dish1", sop, yaw=0)
            att_list.append(placement)
        self.init_mode = Mode.from_list(att_list)
        q = {}
        for robot_name, robot in self.robots.items():
            q[robot_name] = robot.get_joint_angles()
        self.init_config = Config(q)
        self.assign(self.init_mode, self.init_config)

class ProblemKitchen:
    def __init__(self, domain: DomainKitchen):
        self.domain = domain
        self.prob_name = "kitchen_prob1"
        self.objects = {
            "food":["box1", "box2", "box3", "box4"],
            "sink":["sink1"],
            "dish":["dish1"],
            "oven":["oven1"]
        }
        self.init = [
            ("clear", "box1"),
            ("clear", "box2"),
            ("clear", "box3"),
            ("clear", "box4"),
            ("on", "dish1", "box1"),
            ("on", "dish1", "box2"),
            ("on", "dish1", "box3"),
            ("on", "dish1", "box4"),
            ("handempty")
        ]
        self.goal = [ #and
            ("cooked", "box1"),
            ("cooked", "box2"),
            ("cooked", "box3"),
            ("cooked", "box4"),
        ]

    def get_prob_pddl(self, shuffle=False):
        n = "\n"
        object_dict = deepcopy(self.objects)
        init = deepcopy(self.init)
        goal = deepcopy(self.goal)
        if shuffle:
            for obj_list in object_dict.values():
                np.random.shuffle(obj_list)
            np.random.shuffle(init)
            np.random.shuffle(goal)
            
        obj_string = "\n".join([" ".join(objs) + f" - {t}" for t, objs in object_dict.items()])
        init_string = "\n".join(["("+" ".join(pred)+")" if type(pred) is tuple else "("+pred+")" for pred in init])
        goal_string = "\n".join(["("+" ".join(pred)+")" if type(pred) is tuple else "("+pred+")" for pred in goal])
        pddl_string = f"(define {n}(problem {self.prob_name}){n}(:domain {self.domain.domain_name}){n}(:objects {n}{obj_string}){n}(:init {n}{init_string}){n}(:goal {n}(and {n}{goal_string})))"
        return pddl_string
    
    def make_temp_pddl_files(self, path="./temp_pddl", shuffle=False):
        if os.path.exists(path):
            shutil.rmtree(path)
        os.mkdir(path)
        domain_file_path = path + "/domain.pddl"
        problem_file_path = path + "/problem.pddl"
        with open(domain_file_path, "w") as f:
            f.write(self.domain.domain_pddl_string)
        with open(path+"/problem.pddl", "w") as f:
            f.write(self.get_prob_pddl(shuffle=shuffle))
        return domain_file_path, problem_file_path

if __name__ == "__main__":
    dom = DomainKitchen(gui=True)
    prob = ProblemKitchen(dom)
    prob.make_temp_pddl_files()
    
    #input()