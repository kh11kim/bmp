from pathlib import Path

from pybullet_suite import *
from bmp.utils.utils import *
from bmp.base import *
import os
import shutil

##############################
class DomainHanoi(TAMPDomain):
    def __init__(self, gui):
        domain_pddl_path = (Path(__file__).parent / "pddl/domain.pddl").as_posix()
        super().__init__(
            gui, 
            domain_name="HANOI", 
            domain_pddl_path=domain_pddl_path
        )
        self.placeables = {**self.regions, **self.movables}

        # not mandatory
        self.action_info = {
            "geometric": ["pick", "place"],
            #"non-geometric": ["wash", "cook"]
        }
        self.region_types = ["peg"]
        self.movable_types = ["disc"]
        self.predicates = ["on", "handempty", "holding", "clear", "smaller"]

    def set_task_scene(self):
        with no_output():
            with self.world.no_rendering():
                sm = BulletSceneMaker(self.world)
                plane = sm.create_plane(z_offset=-0.4)
                ground = sm.create_table("ground", 2, 2, 0.4) #ground
                hand = Gripper(self.world)

                # env parameter
                p1 = (0.4, 0.2)
                p2 = (0.45, 0.0)
                p3 = (0.4, -0.2)
                h_table = 0.2
                r_peg = 0.015
                h_peg = 0.03
                h_gap = 0.06 #height of each tower is 0.06
                table_gap = 0.01

                # set environment
                envs = {}
                envs["table"] = sm.create_table(
                    "table", 0.4, 1.2, h_table, 
                    x_offset=0.4, y_offset=0, z_offset=0.2)
                
                # set regions
                regions = {}
                names = ["peg1", "peg2", "peg3"]
                positions = [
                    [*p1, h_table+h_peg/2 +table_gap],
                    [*p2, h_table+h_peg/2 +table_gap],
                    [*p3, h_table+h_peg/2 +table_gap]
                ]
                color = [0.7, 0.7, 0.7, 1]
                for idx, name in enumerate(names):
                    regions[name] = sm.create_cylinder( 
                        body_name=name, 
                        radius=r_peg,
                        height=h_peg,
                        mass=1., 
                        pose=Pose(trans=positions[idx]),
                        rgba_color=color)
                
                # set movables
                movables = {}
                names = ["disk1", "disk2", "disk3"] #, ""
                urdf_folder_path = Path(__file__).parent
                urdfs = [
                    "urdf/cylinder1.urdf",
                    "urdf/cylinder2.urdf",
                    "urdf/cylinder3.urdf"
                ]
                positions = [
                    [*p1, h_table+2*h_gap +table_gap],#
                    [*p1, h_table+h_gap+table_gap],#
                    [*p1, h_table+table_gap],
                ]
                for idx, name in enumerate(names):
                    movables[name] = self.world.load_urdf(
                        name=name,
                        urdf_path=(urdf_folder_path / urdfs[idx]).as_posix(),
                        pose=Pose(trans=positions[idx])
                    )

                # set robot
                robots = {"robot":self.world.load_robot("robot", robot_class=Panda)}
                self.world.set_view(eye_point=[1,-0.2,0.8])
        return movables, regions, robots, envs

    def set_tamp_objects(self, movables:Dict, regions:Dict, robots:Dict):
        class side_grasp_gen_fn:
            def __init__(self, body:Movable):
                self.body = body
                lower, upper = body.get_AABB_wrt_obj_frame(output_center_extent=False)
                z_offset = 0.03
                z = upper[-1] - z_offset
                self.grasp_width = 0.02
                self.rot = Rotation.from_euler("xyz",[np.pi/2,np.pi/2,0])
                self.trans = np.array([0,0,z])
            def __call__(self):
                yaw = np.random.uniform(0, np.pi*2)
                rot_yaw = Rotation.from_euler("xyz",[0,0,yaw])
                grasp_pose = Pose(rot=rot_yaw*self.rot, trans=self.trans)
                return Grasp(
                    self.body.name, tf=grasp_pose, width=self.grasp_width,
                    pre_pose_distance=0.1, approach="top"
                )
        def get_sops(body: Body):
            _, (w, l, h) = body.get_AABB_wrt_obj_frame()
            sop = SOP(Rotation.identity(), np.array([0,0,1]), 0)
            sops = [sop]
            return sops
        def get_sssp(body: Body, z_offset: float=0.):
            lower, upper = body.get_AABB_wrt_obj_frame(output_center_extent=False)
            z = upper[-1] + z_offset
            point = np.array([0,0,z])
            return SSSP(point, point)

        self.movables = {}
        for name, body in movables.items():
            sops = get_sops(body)
            sssp = get_sssp(body, -0.06)
            self.movables[name] = Movable.from_body(body, name, sops, sssp)
            self.movables[name].set_grasps(side_grasp_gen_fn(self.movables[name]))
        
        self.regions = {}
        for name, body in regions.items():
            sssp = get_sssp(body)
            self.regions[name] = Region.from_body(body, name, sssp)
        
        self.robots = {}
        for name, robot in robots.items():
            self.robots[name] = robot
    
class ProblemHanoi(TAMPProblem):
    def __init__(self, domain: DomainHanoi):
        super().__init__(
            prob_name="hanoi_prob1",
            domain=domain
        )

    def set_objects(self):
        self.pegs = ["peg1", "peg2", "peg3"]
        self.disks = ["disk1", "disk2", "disk3"]
        self.objects = {
            "peg":self.pegs,
            "disk":self.disks,
        }
    
    def is_placed_peg(self, movable:Movable, region: Region):
        eps = 0.05
        lower1, upper1 = movable.get_AABB()
        lower2, upper2 = region.get_AABB()
        center = movable.get_base_pose().trans
        is_z_contact = upper2[2] - eps <= lower1[2] <= upper2[2] + eps
        is_in_area = np.less_equal(lower2[:2], center[:2]).all() and \
                    np.less_equal(center[:2], upper2[:2]).all()
        return is_z_contact and is_in_area
    
    def is_placed_disc(self, movable:Movable, movable2: Movable):
        if movable == movable2: return False
        
        eps = 0.01
        lower1, upper1 = movable.get_AABB()
        lower2, upper2 = movable2.get_AABB()
        center = movable.get_base_pose().trans
        lower_disc_z_contact = upper2[2]-0.066
        upper_disc_z_contact = lower1[2]
        is_z_contact = lower_disc_z_contact - eps <= upper_disc_z_contact <= lower_disc_z_contact + eps
        is_in_area = np.less_equal(lower2[:2], center[:2]).all() and \
                    np.less_equal(center[:2], upper2[:2]).all()
        return is_z_contact and is_in_area

    def set_init_goal(self):
        hand_clean = [("clear", disk) for disk in self.disks]
        hand_clean += [("handempty")]
        hanoi_disc_constraint = [
            ("smaller", "disk1", "disc2"),
            ("smaller", "disk1", "disk3"),
            ("smaller", "disk2", "disk3"),
            ("smaller", "disc1", "peg1"),
            ("smaller", "disc1", "peg2"),
            ("smaller", "disc1", "peg3"),
            ("smaller", "disc2", "peg1"),
            ("smaller", "disc2", "peg2"),
            ("smaller", "disc2", "peg3"),
            ("smaller", "disc3", "peg1"),
            ("smaller", "disc3", "peg2"),
            ("smaller", "disc3", "peg3"),
        ]
        self.init = [
            *hand_clean,
            *hanoi_disc_constraint,
            ("on", "disk3", "peg1"),
            ("on", "disk2", "disk3"),
            ("on", "disk1", "disk2"),
        ]
        self.goal = [ #and
            ("on", "disk3", "peg3"),
            ("on", "disk2", "disk3"),
            ("on", "disk1", "disk2"),
            *hand_clean
        ]

    def set_init_mode_config(self):
        # mode_init: all movables are placed on the peg3 sequencially
        #list(self.domain.movables.keys())
        children = ["disk3", "disk2", "disk1", ] #, 
        parent_obj = "peg1"
        att_list = []
        for child in children:
            movable = self.domain.movables[child]
            sop = movable.sops[0]
            placement = self.domain.get_current_placement(child, parent_obj, sop)
            att_list.append(placement)
            parent_obj = child
        self.mode_init = Mode.from_list(att_list)
        
        # config_init: robot joint home position
        q = {}
        for robot_name, robot in self.domain.robots.items():
            q[robot_name] = robot.get_joint_angles()
        self.config_init = Config(q)
        self.domain.assign(self.mode_init, self.config_init)


if __name__ == "__main__":
    dom = DomainHanoi(gui=True)
    prob = ProblemHanoi(dom)
    prob.make_temp_pddl_files()
    
    #input()