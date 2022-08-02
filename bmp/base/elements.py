from pybullet_suite import *
from copy import deepcopy
from dataclasses import dataclass, field

PRE_POSE_DISTANCE = 0.05

@dataclass
class SOP:
    rot: Rotation
    axis: np.ndarray # axis of rotation
    height: float # height from sssp

    @property
    def tf(self):
        return Pose(rot=self.rot, trans=[0,0,self.height]).inverse() #TODO:this assumes axis is z

@dataclass
class SSSP:
    lower: np.ndarray
    upper: np.ndarray

@dataclass
class Attachment:
    obj_name: str
    tf: Pose
    parent_name: Optional[str] = field(default_factory=lambda : None)

    @staticmethod
    def get_pre_pose(pose:Pose):
        raise NotImplementedError()

@dataclass
class Grasp(Attachment):
    width: Optional[float] = field(default_factory=lambda : 0)

    @staticmethod
    def get_pre_pose(pose:Pose):
        pre_pose = Pose(trans=[0,0,-PRE_POSE_DISTANCE])
        return pose * pre_pose

@dataclass
class Placement(Attachment):
    point: Optional[np.ndarray] = field(default_factory=lambda : None)
    sop: Optional[SOP] = field(default_factory=lambda : None)
    yaw: Optional[float] = field(default_factory=lambda : None)
    obj_pose: Optional[Pose] = field(default_factory=lambda : None)

    @staticmethod
    def get_pre_pose(pose: Pose):
        pre_pose = Pose(trans=[0,0,+PRE_POSE_DISTANCE])
        return pre_pose * pose

    @classmethod
    def from_point_and_sop(
        cls, 
        movable_name: str,
        placeable_name: str,
        placeable_pose: Pose,
        point: np.ndarray,
        sop: SOP,
        yaw: Optional[float]=None #if None, random yaw
    ):
        if yaw is None:
            yaw = np.random.uniform(0, np.pi*2)
        trans = point + np.array([0,0,sop.height])
        rot = Rotation.from_euler("xyz",[0,0,yaw]) * sop.rot
        obj_pose = Pose(rot=rot, trans=trans)
        tf = obj_pose.inverse() * placeable_pose
        return Placement(
            obj_name=movable_name, 
            tf=tf, 
            point=point, 
            sop=sop, 
            yaw=yaw,
            obj_pose=obj_pose,
            parent_name=placeable_name)

class Mode:
    """ All attachments of movables
    """
    def __init__(self, atts: Dict[str, Attachment]):
        self._attachments: Dict[str, Attachment] = atts
        self.mode_key = self.get_mode_key(atts)
        
    @property
    def attachments(self):
        return deepcopy(self._attachments)

    @attachments.setter
    def attachments(self, atts:Dict[str, Attachment]):
        self._attachments = atts
        self.mode_key = self.get_mode_key(atts)

    def get_mode_key(self, atts:Dict[str, Attachment]):
        mode_key = {}
        for obj, att in atts.items():
            mode_key[obj] = (att.parent_name, att.tf.as_1d_numpy())
        return mode_key

    @classmethod
    def from_list(cls, att_list: List[Attachment]):
        atts = {}
        for att in att_list:
            obj = att.obj_name
            atts[obj] = att
        return cls(atts)
            
    def copy(self):
        return Mode(deepcopy(self.attachments))

    def __eq__(self, other:"Mode"):
        for obj in self.mode_key:
            parent_name1, tf_vec1 = self.mode_key[obj]
            parent_name2, tf_vec2 = other.mode_key[obj]
            if (parent_name1 != parent_name2) | \
                (not np.array_equal(tf_vec1, tf_vec2)):
                return False
        return True


class TAMPObject(Body):
    """Wrapper function of Body object for TAMP
    """
    def __init__(
        self, 
        physics_client: BulletClient, 
        body_uid: int,
        name: str,
        sssp: SSSP
    ):
        super().__init__(
            physics_client,
            body_uid,
        )
        self.name = name
        self.is_movable = None
        self.sssp = sssp # surfaces supporting stable placement
    
class Movable(TAMPObject):
    def __init__(
        self,
        physics_client: BulletClient,
        body_uid: int,
        name: str,
        grasps: List[Grasp],
        sops: List[SOP],
        sssp: SSSP
    ):
        super().__init__(
            physics_client=physics_client,
            body_uid=body_uid,
            name=name,
            sssp=sssp
        )
        self.grasps: List[Grasp] = grasps
        # Stable Object Poses : (Rotation, axis of rotation: R3, distance to a sssp: R1)
        self.sops: List[SOP] = sops
        
    @classmethod
    def from_body(cls, body: Body, name: str, grasps, sops, sssp):
        return cls(body.physics_client, body.uid, name, grasps, sops, sssp)
    
    def sample_grasp(self):
        i = np.random.randint(0, len(self.grasps))
        return deepcopy(self.grasps[i])
    
    def sample_sop(self):
        """Stable object pose
        """
        i = np.random.randint(0, len(self.sops))
        return deepcopy(self.sops[i])


class Region(TAMPObject):
    def __init__(
        self,
        physics_client: BulletClient,
        body_uid: int,
        name: str,
        sssp: SSSP
    ):
        super().__init__(
            physics_client=physics_client,
            body_uid=body_uid,
            name=name,
            sssp=sssp
        )
        
    @classmethod
    def from_body(cls, body: Body, name: str, sssp: SSSP):
        return cls(body.physics_client, body.uid, name, sssp)

@dataclass
class Config:
    q: Dict[str, np.ndarray]