from pathlib import Path

from pybullet_suite import *
from bmp.utils.utils import *
from bmp.base import *
import os


##############################
class DomainSorting(TAMPDomain):
    def __init__(self, gui, num_box=3):
        domain_pddl_path = Path(__file__).parent / Path("pddl/domain.pddl")
        self.num_box=num_box
        super().__init__(
            gui,
            domain_name="Sorting",
            domain_pddl_path=domain_pddl_path
        )
        self.placeables = {**self.regions}
        

    def set_task_scene(self):
        with no_output():
            with self.world.no_rendering(activate=False):
                sm = BulletSceneMaker(self.world)
                plane = sm.create_plane(z_offset=-0.4)
                hand = Gripper(self.world)

                # set environment
                envs = {}
                table_x_offset = 0.55
                table_x_offset_gray = 0.45
                envs["ground"] = sm.create_table("ground", 2, 2, 0.4) #ground
                
                #regions
                regions = {}
                table_width = 0.6
                regions["table_gray1"] = sm.create_table(
                    "table_gray1", 0.35, table_width, 0., 
                    x_offset=table_x_offset_gray, y_offset=0, z_offset=0.2,
                    rgba=[0.9,0.9,0.9,0.8])

                regions["table_red"] = sm.create_table(
                    "table_red", table_width, 0.4, 0.2, 
                    x_offset=0, y_offset=table_x_offset, z_offset=0.2,
                    rgba=[1,0,0,0.8])
                
                regions["table_green"] = sm.create_table(
                    "table_green", table_width, 0.4, 0.2, 
                    x_offset=0, y_offset=-table_x_offset, z_offset=0.2,
                    rgba=[0,1,0,0.8])

                
                # set movables
                movables = {}
                w, h = 0.03, 0.07
                half_extents = [w/2, w/2, h/2]
                gap = w+0.02
                num_box_per_color = 5
                num_box_obs = 10
                names = ["red", "green", "obs"]

                x_offset = table_x_offset_gray + 0.12
                colors = [
                    [1, 0, 0, 1],
                    [0, 1, 0, 1],
                    [0.6, 0.6, 0.6, 1],
                ]
                xz_pos = [
                    [table_x_offset_gray, 0.2+h/2],
                    [table_x_offset_gray-gap, 0.2+h/2],
                    [table_x_offset_gray+gap, 0.2+h/2],
                ]

                for idx, name in enumerate(names):
                    xz = xz_pos[idx]
                    num_box = num_box_per_color if name in ["red", "green"] else num_box_obs
                    for i in range(num_box):
                        box_name = name+str(i+1)
                        position = [xz[0], i*gap-0.27, xz[1]]
                        movables[box_name] = sm.create_box(
                            body_name=box_name, 
                            half_extents=half_extents, 
                            mass=1., 
                            position=position,
                            rgba_color=colors[idx])
                
                # xz_pos = [
                #     [-table_x_offset, 0.2+h/2],
                #     [-table_x_offset-gap, 0.2+h/2],
                #     [-table_x_offset+gap, 0.2+h/2],
                # ]

                # for idx, name in enumerate(names):
                #     xz = xz_pos[idx]
                #     for i in range(num_box_per_color):
                #         box_name = name+str(i+1+num_box_per_color)
                #         position = [xz[0], i*gap-0.1, xz[1]]
                #         movables[box_name] = sm.create_box(
                #             body_name=box_name, 
                #             half_extents=half_extents, 
                #             mass=1., 
                #             position=position,
                #             rgba_color=colors[idx])
            
                
                
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
    
class ProblemSorting(TAMPProblem):
    def __init__(self, domain: DomainSorting):
        super().__init__(
            prob_name="nonmonotonic_prob1",
            domain=domain
        )
        self.randomize_block_pos()
    
    def randomize_block_pos(self):
        for i in range(100):
            for obj in self.mode_init.attachments.keys():
                parent = self.mode_init.attachments[obj].parent_name
                self.mode_init.attachments[obj] = self.domain.sample_attachment(obj, parent)
            if not self.domain.is_collision(self.mode_init, config=self.config_init):
                break
        


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
        self.mode_goal = {}
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
    dom = DomainSorting(gui=True)
    prob = ProblemSorting(dom)
    input()