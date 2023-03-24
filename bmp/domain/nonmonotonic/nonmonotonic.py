from pathlib import Path

from pybullet_suite import *
from bmp.utils.utils import *
from bmp.base import *
import os


##############################
class DomainNonmonotonic(TAMPDomain):
    def __init__(self, gui, num_box=3):
        domain_pddl_path = Path(__file__).parent / Path("pddl/domain.pddl")
        self.num_box=num_box
        super().__init__(
            gui,
            domain_name="Nonmonotonic",
            domain_pddl_path=domain_pddl_path
        )
        self.placeables = {**self.regions}

    def set_task_scene(self):
        with no_output():
            with self.world.no_rendering(activate=True):
                sm = BulletSceneMaker(self.world)
                plane = sm.create_plane(z_offset=-0.4)
                hand = Gripper(self.world)

                # set environment
                envs = {}
                table_x_offset = 0.4
                envs["table"] = sm.create_table(
                    "table", 0.4, 1.2, 0.2, 
                    x_offset=table_x_offset, y_offset=0, z_offset=0.2)
                envs["ground"] = sm.create_table("ground", 2, 2, 0.4) #ground
                
                if self.num_box == 2:
                    # set regions
                    regions = {}
                    names = ["plate_red", "plate_green"]
                    w, h = 0.3, 0.02
                    w1 = w
                    gap_plate = 0.32
                    half_extents = [
                        [w/2, w/2, h/2],
                        [w1/2, w1/2, h/2],
                        [w1/2, w1/2, h/2],
                    ]
                    positions = [
                        [table_x_offset, +gap_plate/2, 0.2+0.01],
                        [table_x_offset, -gap_plate/2, 0.2+0.01],
                        #[table_x_offset, -gap_plate, 0.2+0.01]
                    ]
                    colors = [
                        [1, 0, 0, 1],
                        [0, 1, 0, 1],
                        [0, 0, 1, 1]
                    ]
                    for idx, name in enumerate(names):
                        regions[name] = sm.create_box(
                            body_name=name, 
                            half_extents=half_extents[idx], 
                            mass=1., 
                            position=positions[idx],
                            rgba_color=colors[idx])
                    
                    # set movables
                    movables = {}
                    w, h = 0.03, 0.07
                    half_extents = [w/2, w/2, h/2]
                    gap = w+0.02
                    names = ["red1", "green1"]#, "blue1", "blue2"]

                    x_offset = table_x_offset + 0.12
                    positions = [
                        [x_offset, -gap_plate/2, 0.2+0.02+h/2],
                        [x_offset, +gap_plate/2, 0.2+0.02+h/2],
                    ]
                    colors = [
                        [1, 0, 0, 1],
                        [0, 1, 0, 1],
                        [0, 0, 1, 1],
                    ]
                    for idx, name in enumerate(names):
                        movables[name] = sm.create_box(
                            body_name=name, 
                            half_extents=half_extents, 
                            mass=1., 
                            position=positions[idx],
                            rgba_color=colors[idx])
                
                if self.num_box == 3:
                    # set regions
                    regions = {}
                    names = ["plate_red", "plate_green", "plate_blue"]
                    w, h = 0.3, 0.02
                    w1 = w
                    gap_plate = 0.32
                    half_extents = [
                        [w/2, w/2, h/2],
                        [w1/2, w1/2, h/2],
                        [w1/2, w1/2, h/2],
                    ]
                    positions = [
                        [table_x_offset, 0, 0.2+0.01],
                        [table_x_offset, +gap_plate, 0.2+0.01],
                        [table_x_offset, -gap_plate, 0.2+0.01]
                    ]
                    colors = [
                        [1, 0, 0, 1],
                        [0, 1, 0, 1],
                        [0, 0, 1, 1]
                    ]
                    for idx, name in enumerate(names):
                        regions[name] = sm.create_box(
                            body_name=name, 
                            half_extents=half_extents[idx], 
                            mass=1., 
                            position=positions[idx],
                            rgba_color=colors[idx])
                    
                    # set movables
                    movables = {}
                    w, h = 0.03, 0.07
                    half_extents = [w/2, w/2, h/2]
                    gap = w+0.02
                    names = ["red1", "green1", "blue1"]

                    x_offset = table_x_offset + 0.12
                    positions = [
                        [x_offset, +gap_plate, 0.2+0.02+h/2],
                        [x_offset, -gap_plate, 0.2+0.02+h/2],
                        [x_offset, +0, 0.2+0.02+h/2],
                    ]
                    colors = [
                        [1, 0, 0, 1],
                        [0, 1, 0, 1],
                        [0, 0, 1, 1],
                    ]
                    for idx, name in enumerate(names):
                        movables[name] = sm.create_box(
                            body_name=name, 
                            half_extents=half_extents, 
                            mass=1., 
                            position=positions[idx],
                            rgba_color=colors[idx])
                
                
                #obstacle
                w, h = 0.03, 0.15
                half_extents = [w/2, w/2, h/2]
                names = [f"obs{i+1}" for i in range(len(names))]
                color = [0.6, 0.6, 0.6, 1]
                for idx, name in enumerate(names):
                    position = positions[idx]
                    position[0] -= 0.04
                    movables[name] = sm.create_box(
                        body_name=name, 
                        half_extents=half_extents, 
                        mass=1., 
                        position=position,
                        rgba_color=color)
                
                
                # set robot
                robots = {"robot":self.world.load_robot("robot", robot_class=Panda)}
                
        self.world.set_view(eye_point=[1.2,-0.2,0.7])
        self.world.wait_for_rest()
        return movables, regions, robots, envs

    def set_tamp_objects(self, movables:Dict, regions:Dict, robots:Dict):
        def get_top_grasps(body: Body):
            max_grasp_width = 0.15
            grasp_depth = 0.015
            _, (w, l, h) = body.get_AABB_wrt_obj_frame()
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
            grasps = [Grasp(name, tf=tf, width=width, pre_pose_distance=0.15) 
                for tf, width in zip(grasp_poses, grasp_widths)]
            return grasps
        def get_sops(body: Body):
            _, (w, l, h) = body.get_AABB_wrt_obj_frame()
            sop = SOP(Rotation.identity(), np.array([0,0,1]), h/2)
            sops = [sop]
            return sops
        def get_sssp(body: Body, z_offset: float=0.):
            lower, upper = body.get_AABB_wrt_obj_frame(output_center_extent=False)
            lower, upper = body.get_AABB_wrt_obj_frame(output_center_extent=False)
            lower[-1] = upper[-1]
            return SSSP(lower, upper)

        self.movables = {}
        for name, body in movables.items():
            grasps = get_top_grasps(body)
            sops = get_sops(body)
            sssp = get_sssp(body)
            self.movables[name] = Movable.from_body(body, name, sops, sssp)
            self.movables[name].set_grasps(grasps)
        
        self.regions = {}
        for name, body in regions.items():
            sssp = get_sssp(body)
            self.regions[name] = Region.from_body(body, name, sssp)
        
        self.robots = {}
        for name, robot in robots.items():
            self.robots[name] = robot
    
class ProblemNonmonotonic(TAMPProblem):
    def __init__(self, domain: DomainNonmonotonic):
        super().__init__(
            prob_name="nonmonotonic_prob1",
            domain=domain
        )

    def set_objects(self):\
        pass
        # self.boxes = ["box1", "box2", "box3"]
        # self.plates = ["plate1", "plate2"]
        # self.objects = {
        #     "box":self.boxes,
        #     "plate":self.plates,
        # }
    
    def is_placed(self, movable:Movable, region: Region):
        eps = 0.01
        lower1, upper1 = movable.get_AABB()
        lower2, upper2 = region.get_AABB()
        center = movable.get_base_pose().trans
        is_z_contact = upper2[2] - eps <= lower1[2] <= upper2[2] + eps
        is_in_area = np.less_equal(lower2[:2], center[:2]).all() and \
                    np.less_equal(center[:2], upper2[:2]).all()
        return is_z_contact and is_in_area

    def set_init_goal(self):
        pass
        #hand_clean = [("clear", disk) for disk in self.disks]
        # hand_empty = [("handempty")]
        # self.init = [
        #     hand_empty,
        #     *hanoi_disc_constraint,
        #     ("on", "disk3", "peg1"),
        #     ("on", "disk2", "disk3"),
        #     ("on", "disk1", "disk2"),
        # ]
        # self.goal = [ #and
        #     ("on", "disk3", "peg3"),
        #     ("on", "disk2", "disk3"),
        #     ("on", "disk1", "disk2"),
        #     *hand_clean
        # ]

    def set_init_mode_config(self):
        # mode_init: all movables are placed on the peg3 sequencially
        att_list = []
        for movable in self.domain.movables.values():
            for obj in self.domain.objects.values():
                if self.is_placed(movable, obj):
                    sop = movable.sops[0]
                    placement = self.domain.get_current_placement(movable.name, obj.name, sop)
                    att_list += [placement]
                    break
        
        self.mode_init = Mode.from_list(att_list)
        
        # config_init: robot joint home position
        q = {}
        for robot_name, robot in self.domain.robots.items():
            q[robot_name] = robot.get_joint_angles()
        self.config_init = Config(q)
        self.domain.assign(self.mode_init, self.config_init)

if __name__ == "__main__":
    dom = DomainNonmonotonic(gui=True)
    input()
    #problem = ProblemKitchen(dom)
    
    #problem.make_temp_pddl_files()
    
    #input()