from pathlib import Path

from pybullet_suite import *
#from bmp.utils.utils import *
from bmp.base import *

##############################
class DomainRegrasping(TAMPDomain):
    def __init__(self, gui, num_dice=1):
        domain_pddl_path = (Path(__file__).parent / "pddl/domain.pddl").as_posix()
        self.dice_urdf_path = (Path(__file__).parent / "dice/urdf/dice.urdf").as_posix()
        self.dice_grasp_set_path = (Path(__file__).parent / "dice/grasp_set.npz").as_posix()
        self.num_dice = num_dice
        super().__init__(
            gui, 
            domain_name="PICK-AND-PLACE", 
            domain_pddl_path=domain_pddl_path
        )
        self.placeables = self.regions
                
        # not mandatory
        self.action_info = {
            "geometric": ["pick", "place"],
        }
        self.region_types = ["plate"]
        self.movable_types = ["box"]
        self.predicates = ["handempty"]

    def set_task_scene(self):
        with no_output(no_output=True):
            with self.world.no_rendering(activate=False):
                
                sm = BulletSceneMaker(self.world)
                plane = sm.create_plane(z_offset=-0.4)
                ground = sm.create_table("ground", 2, 2, 0.4) #ground
                hand = Gripper(self.world)

                # set environment
                envs = {}
                envs["ground"] = sm.create_table("ground", 2, 2, 0.4) #ground
                envs["table_big"] = sm.create_table(
                    "table_big", 0.4, 1.2, 0.2, 
                    x_offset=0.4, y_offset=0, z_offset=0.2)
                
                #regions
                regions = {}
                w, h = 0.22, 0.02
                half_extents = [w/2, w/2, h/2]
                positions = [0.4, 0, 0.2+0.01]
                color = [0.6, 0.6, 0.6, 1]
                regions["table"] = sm.create_box(
                    body_name="table", 
                    half_extents=half_extents, 
                    mass=1., 
                    position=positions,
                    rgba_color=color)
                                
                # set movables
                movables = {}
                d = 0.036
                gap = 0.1
                for i in range(self.num_dice):
                    dice_name = "dice" + str(i+1)
                    pose = Pose(trans=[0.4, 0+gap*i, 0.2+0.02+d/2])
                    movables[dice_name] = self.world.load_urdf(
                        dice_name, self.dice_urdf_path, pose
                    )


                # set robot
                robots = {"robot":self.world.load_robot("robot", robot_class=Panda)}
                self.world.set_view(eye_point=[1,-0.2,0.8])
                self.world.wait_for_rest()
        return movables, regions, robots, envs

    def set_tamp_objects(self, movables:Dict, regions:Dict, robots:Dict):
        def get_all_grasps(body: Body):
            max_grasp_width = 0.15
            grasp_set = np.load(self.dice_grasp_set_path, allow_pickle=True)["grasp_set"]
            width = 0.036
            # grasp_depth = 0.015
            # _, (w, l, h) = body.get_AABB_wrt_obj_frame()
            # rot = Rotation.from_euler("xyz",[np.pi,0,0])
            # trans = [0,0,h/2-grasp_depth]
            # grasp_poses = []
            # grasp_widths = []
            grasps = []
            for tf in grasp_set:
                grasps += [Grasp(name, tf=tf, width=width)]
            return grasps
            # if w <= max_grasp_width:
            #     grasp_poses += [
            #         Pose(rot=rot, trans=trans),
            #         Pose(rot=Rotation.from_euler("xyz",[0,0,np.pi])*rot,trans=trans)
            #     ]
            #     grasp_widths += [w, w]
            # if l <= max_grasp_width:
            #     grasp_poses += [
            #         Pose(rot=Rotation.from_euler("xyz",[0,0,np.pi/2])*rot,trans=trans),
            #         Pose(rot=Rotation.from_euler("xyz",[0,0,np.pi*3/2])*rot,trans=trans)
            #     ]
            # #     grasp_widths += [l, l]
            # grasps = [Grasp(name, tf=tf, width=width) 
            #     for tf, width in zip(grasp_poses, grasp_widths)]
            # return grasps

        def get_all_sops(body: Body):
            h = 0.036
            rots = [
                Rotation.identity(),
                Rotation.from_euler("zyx", [0,0,np.pi/2]),
                Rotation.from_euler("zyx", [0,0,-np.pi/2]),
                Rotation.from_euler("zyx", [0,0,np.pi]),
                Rotation.from_euler("zxy", [0,0,np.pi/2]),
                Rotation.from_euler("zxy", [0,0,-np.pi/2])
            ]
            sops = []
            for rot in rots:
                sops += [SOP(rot, np.array([0,0,1]), h/2)]
            return sops
        def get_sssp(body: Body, z_offset: float=0.):
            lower, upper = body.get_AABB_wrt_obj_frame(output_center_extent=False)
            lower, upper = body.get_AABB_wrt_obj_frame(output_center_extent=False)
            lower[-1] = upper[-1]
            return SSSP(lower, upper)

        self.movables = {}
        for name, body in movables.items():
            grasps = get_all_grasps(body)
            sops = get_all_sops(body)
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
    
class ProblemRegrasping(TAMPProblem):
    def __init__(self, domain: DomainRegrasping):
        super().__init__(
            prob_name="packing_prob1",
            domain=domain
        )

    def set_objects(self):
        self.boxes = ["box1", "box2", "box3"]
        self.plates = ["plate1", "plate2"]
        self.objects = {
            "box":self.boxes,
            "plate":self.plates,
        }
    
    def set_init_goal(self):
        #set goal
        mode_goal = deepcopy(self.mode_init)
        table = self.domain.regions["table"]
        dice_gap = 0.1
        for i, (dice_name, dice) in enumerate(self.domain.movables.items()):
            sop = dice.sops[3] #backward
            point = np.array([0.4, 0+i*0.1, 0.2+0.02])
            placement = Placement.from_point_and_sop(
                dice_name, "table", table.get_base_pose(),
                point, sop)
            mode_goal.set_attachment(placement)
            self.mode_goal = mode_goal
        self.domain.assign(mode_goal)

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
    dom = DomainRegrasping(gui=True)
    prob = ProblemRegrasping(dom)
    #prob.make_temp_pddl_files()
    
    input()