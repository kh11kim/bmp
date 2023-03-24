from pathlib import Path

from pybullet_suite import *
#from bmp.utils.utils import *
from bmp.base import *

##############################
class DomainPacking(TAMPDomain):
    def __init__(self, gui):
        domain_pddl_path = (Path(__file__).parent / "pddl/domain.pddl").as_posix()
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
            with self.world.no_rendering(activate=True):
                num_box = 5
                sm = BulletSceneMaker(self.world)
                plane = sm.create_plane(z_offset=-0.4)
                ground = sm.create_table("ground", 2, 2, 0.4) #ground
                hand = Gripper(self.world)

                # set environment
                envs = {}
                envs["table"] = sm.create_table(
                    "table", 0.4, 1.2, 0.2, 
                    x_offset=0.4, y_offset=0, z_offset=0.2)
                envs["ground"] = sm.create_table("ground", 2, 2, 0.4) #ground
                
                
                # set regions
                regions = {}
                names = ["plate1", "plate2"]
                w, h = 0.22, 0.02
                w1 = w#0.1
                half_extents = [
                    [w/2, w/2, h/2],
                    [w1/2, w1/2, h/2]
                ]
                positions = [
                    [0.4, +0.3, 0.2+0.01],
                    [0.4, -0.3, 0.2+0.01]
                ]
                colors = [
                    [0.6, 0.6, 0.6, 1],
                    [0.6, 0.6, 0.6, 1],
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
                w, h = 0.06, 0.07
                half_extents = [w/2, w/2, h/2]
                gap = w+0.03
                names = [f"box{i+1}" for i in range(5)]
                positions = [
                    [0.4, 0-0.3, 0.2+0.02+0.05],
                    [0.4, -gap-0.3, 0.2+0.02+0.05],
                    [0.4, +gap-0.3, 0.2+0.02+0.05],
                    [0.4-gap, 0-0.3, 0.2+0.02+0.05],
                    [0.4+gap, 0-0.3, 0.2+0.02+0.05]
                ]
                colors = [
                    [1, 0, 0, 1],
                    [0, 1, 0, 1],
                    [0, 0, 1, 1],
                    [0.7, 0.7, 0, 1],
                    [0.7, 0, 0.7, 1],
                ]
                for idx, name in enumerate(names[:num_box]):
                    movables[name] = sm.create_box(
                        body_name=name, 
                        half_extents=half_extents, 
                        mass=1., 
                        position=positions[idx],
                        rgba_color=colors[idx])

                # set robot
                robots = {"robot":self.world.load_robot("robot", robot_class=Panda)}
                self.world.set_view(eye_point=[1,-0.2,0.8])
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
            grasps = [Grasp(name, tf=tf, width=width) 
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
    
class ProblemPacking(TAMPProblem):
    def __init__(self, domain: DomainPacking):
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
    dom = DomainPacking(gui=True)
    #prob = ProblemUnpacking(dom)
    #prob.make_temp_pddl_files()
    
    input()