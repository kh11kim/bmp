from pybullet_suite import *
#from .elements import *
from .elements import *

class Checker:
    def __init__(self, gui:bool = False):
        self.world = BulletWorld(gui=gui)
        self.sm = BulletSceneMaker(self.world)
        self.robot: Panda = self.world.load_robot("robot", robot_class=Panda)
        self.hand = Gripper(self.world)
        self.hand2 = Gripper(self.world)
        self.hand.reset(self.hand.remove_pose, "hand")
        self.hand2.reset(self.hand2.remove_pose, "hand2")
        self.ground = self.sm.create_table("ground", 2, 2, 0.5)
        self.ground_height = 0.25
    
    def is_tool_collision(self, att1: Attachment, att2: Attachment) -> bool:
        if type(att1) != type(att2):
            if type(att1) is Grasp:
                grasp, placement = att1, att2
            else:
                grasp, placement = att2, att1
            return self.is_gp_collision(grasp, placement)
        elif type(att1) is Grasp:
            return self.is_gg_collision(att1, att2)
        else:
            raise ValueError()
    
    def check_edge_ik(self, att1: Attachment, att2: Attachment, robot_base_pose: Pose=Pose.identity()) -> bool:
        if type(att1) != type(att2):
            if type(att1) is Placement:
                obj_pose = att1.obj_pose
                grasp = att2
            else:
                obj_pose = att2.obj_pose
                grasp = att1
            ee_pose = obj_pose * grasp.tf
            ee_pre_pose = grasp.get_pre_pose(ee_pose)
            grasp_pose_sim = robot_base_pose.inverse() * ee_pre_pose
            q_pre_ik = self.robot.inverse_kinematics(pose=grasp_pose_sim)
            if q_pre_ik is not None:
                return (q_pre_ik, ee_pose)
            return None
        elif type(att1) is Grasp:
            return None
        else:
            raise ValueError()
        
    def get_ground_pose(self, tf: Pose) -> Pose:
        return tf * Pose(trans=[0,0,-self.ground_height])

    def is_ik_feasible(self, ee_pose: Pose, robot_base_pose: Pose = Pose.identity()):
        grasp_pose = robot_base_pose.inverse() * ee_pose
        q = self.robot.inverse_kinematics(pose=grasp_pose)
        if q is not None:
            return True
        return False

    def is_gp_collision(self, g: Grasp, p: Placement):
        result = False
        self.hand.reset(g.tf)
        ground_pose = self.get_ground_pose(p.sop.tf)
        self.ground.set_base_pose(ground_pose)
        if self.world.is_body_pairwise_collision(
            "hand", ["ground"]):
            result = True
        self.hand.remove()
        return result
    
    def is_gg_collision(self, g1: Grasp, g2: Grasp):
        result = False
        self.hand.reset(g1.tf)
        self.hand2.reset(g2.tf, "hand2")
        if self.world.is_body_pairwise_collision(
            "hand", ["hand2"], d=0
        ):
            result = True
        self.hand.remove()
        self.hand2.remove()
        return result