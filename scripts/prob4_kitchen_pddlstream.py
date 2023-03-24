from bmp.domain.kitchen import *
from pddlstream.language.constants import print_solution, PDDLProblem
from pddlstream.utils import read, get_file_path
from pddlstream.language.generator import from_fn, from_test
from pddlstream.algorithms.meta import solve, create_parser
from bmp.planner.rrt import *
from time import sleep

def is_placed(movable:Movable, region: Region):
    eps = 0.01
    lower1, upper1 = movable.get_AABB()
    lower2, upper2 = region.get_AABB()
    center = movable.get_base_pose().trans
    is_z_contact = upper2[2] - eps <= lower1[2] <= upper2[2] + eps
    is_in_area = np.less_equal(lower2[:2], center[:2]).all() and \
                np.less_equal(center[:2], upper2[:2]).all()
    return is_z_contact and is_in_area

def pddlstream_from_problem(domain: DomainKitchen, teleport=False):
    domain_pddl = read(get_file_path(__file__, '../examples/panda_kitchen/domain.pddl'))
    stream_pddl = read(get_file_path(__file__, '../examples/panda_kitchen/stream.pddl'))
    constant_map = {}
    #is_placed(domain.objects["box1"], domain.objects["dish1"])
    panda = domain.robots["robot"]
    q = {"robot":panda.get_joint_angles()}
    config_init = Config(q)
    init = [
        ('CanMove',),
        ('Conf', config_init),
        ("AtConf", config_init),
        ("HandEmpty",)
    ]
    for movable in domain.movables.values():
        init += [
            ('Graspable', movable),
        ]
        for region in domain.regions.values():
            init += [
                ('Stackable', movable, region)
            ]
            if is_placed(movable, region):
                sop = movable.sops[0]
                placement = domain.get_current_placement(movable.name, region.name, sop)
                init += [('Pose', movable, placement)]
                init += [('AtPose', movable, placement)]
                init += [('Supported', movable, placement, region)]
                
    for name, region in domain.regions.items():
        if "sink" in name:
            init += [("Sink", region)]
        if "oven" in name:
            init += [("Oven", region)]
    
    goal = (
        "and",
        ("AtConf", config_init),
        ("Cooked", domain.objects["box1"]),
        ("Cooked", domain.objects["box2"]),
        ("Cooked", domain.objects["box3"]),
        ("Cooked", domain.objects["box4"]),
        ("Cooked", domain.objects["box5"]),
    )

    stream_map = {
        "sample-pose": from_fn(get_placement_fn(domain)),
        "sample-grasp": from_fn(get_grasp_fn(domain)),
        "inverse-kinematics": from_fn(get_ikik_fn(domain)),
        "plan-free-motion": from_fn(get_free_motion_fn(domain, teleport)),
        "plan-holding-motion": from_fn(get_holding_motion_fn(domain, teleport)),
        "plan-approach-motion": from_fn(get_approach_motion_fn(domain, teleport)),

        "test-cfree-pose-pose": from_test(get_cfree_pose_pose_test(domain)),
        "test-cfree-approach-pose": from_test(get_cfree_approach_pose_test(domain)),
        "test-cfree-traj-pose": from_test(get_cfree_traj_pose_test(domain)),
        'Distance': distance_fn,
    }

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

def distance_fn(q1:Config, q2:Config):
    distance = np.linalg.norm(q1.q - q2.q)
    return distance

def get_placement_fn(domain:TAMPDomain):
    def fn(body:Movable, support:TAMPObject):
        placement = domain.sample_placement(body.name, support.name)
        return (placement,)
    return fn

def get_grasp_fn(domain:TAMPDomain):
    robot_name = list(domain.robots.keys())[0]
    def fn(body:Movable):
        grasp = domain.sample_grasp(body.name, robot_name)
        return (grasp,)
    return fn

def get_ikik_fn(domain:TAMPDomain):
    robot_name = list(domain.robots.keys())[0]
    robot: Panda = domain.robots[robot_name]
    def fn(body:Movable, placement:Placement, grasp:Grasp, fluents=[]):
        parent_name = placement.parent_name
        if parent_name in domain.regions:
            parent_pose = domain.objects[parent_name].get_base_pose()
            obj_pose = parent_pose * placement.tf.inverse()
            grasp_pose = obj_pose * grasp.tf
            grasp_pre_pose = grasp.get_pre_pose(grasp_pose)
            q = robot.inverse_kinematics(pose=grasp_pose)
            if q is None: return None
            #else: q = Config(q={robot_name:q})
            q_pre = robot.inverse_kinematics(pose=grasp_pre_pose)
            if q_pre is None: return None
            else: q_pre = Config(q={robot_name:q_pre})
            
            return (q_pre,)
        else:
            raise Exception()
    return fn

def get_free_motion_fn(domain:TAMPDomain, teleport:bool = True):
    robot_name = list(domain.robots.keys())[0]
    def fn(conf1: Config, conf2: Config, fluents=[]):
        if teleport:
            path = [conf1, conf2]
        else:
            atts = {}
            for fluent in fluents:
                if fluent[0] == "atpose":
                    movable:Movable = fluent[1]
                    placement = fluent[2]
                    atts[movable.name] = placement
            mode = Mode(atts)
            if domain.is_collision(mode, conf1): return None
            #planning in mode
            mp = BiRRT2()
            path = mp.plan([robot_name], conf1, conf2, mode, domain)
            if path is None: return None
        return (path,)
    return fn

def get_holding_motion_fn(domain:TAMPDomain, teleport:bool = True):
    robot_name = list(domain.robots.keys())[0]
    def fn(conf1: Config, conf2: Config, body, grasp: Grasp, fluents=[]):
        if teleport:
            path = [conf1, conf2]
        else:
            atts = {grasp.obj_name: grasp}
            for fluent in fluents:
                if fluent[0] == "atpose":
                    movable:Movable = fluent[1]
                    placement = fluent[2]
                    atts[movable.name] = placement
            mode = Mode(atts)
            if domain.is_collision(mode, conf1): return None
            #planning in mode
            mp = BiRRT2()
            path = mp.plan([robot_name], conf1, conf2, mode, domain)
            if path is None: return None
        return (path,)
    return fn

def get_approach_motion_fn(domain:TAMPDomain, teleport:bool=False):
    robot_name = list(domain.robots.keys())[0]
    robot = domain.robots[robot_name]
    def fn(body, placement: Placement, grasp: Grasp, conf1: Config, fluents=[]):
        if teleport:
            path = [conf1, conf1]
        else:
            atts = {}
            for fluent in fluents:
                if fluent[0] == "atpose" or fluent[0] == "atgrasp":
                    movable:Movable = fluent[1]
                    att = fluent[2]
                    atts[movable.name] = att
            mode = Mode(atts)
            #planning in mode
            q_pre = conf1.q[robot_name]
            pre_pose = robot.forward_kinematics(q_pre)
            pose_target = grasp.pre_pose.inverse() * pre_pose
            mp = BiRRT2()
            if domain.is_collision(mode, conf1): return None
            path = mp.check_mode_switch(
                [robot_name], conf1, pose_target, mode, domain)
            if path is None: return None
            gp = {"grasp":grasp, "placement":placement}
            command = (path, gp, path[::-1])
        return (command,)
    return fn

def get_cfree_pose_pose_test(domain:TAMPDomain):
    robot_name = list(domain.robots.keys())[0]
    #robot = domain.robots[robot_name]
    def test(body:Body, placement:Placement, body2:Body, placement2:Placement):
        if domain.is_collision_between_two_placement(placement, placement2):
            return False
        return True
    return test

def get_cfree_approach_pose_test(domain:TAMPDomain):
    robot_name = list(domain.robots.keys())[0]
    robot = domain.robots[robot_name]
    def test(
        body:Body, placement:Placement, grasp:Grasp, 
        body2:Body, placement2:Placement
    ):
        if domain.is_collision_between_pgp(
            placement, grasp, placement2):
            return False
        return True
    return test

def get_cfree_traj_pose_test(domain:TAMPDomain):
    robot_name = list(domain.robots.keys())[0]
    robot = domain.robots[robot_name]
    def test(
        traj:List[Config], body:Body, placement:Placement
    ):
        parent_name = placement.parent_name
        parent_pose = domain.regions[parent_name].get_base_pose()
        obj_pose = parent_pose * placement.tf.inverse()
        body.set_base_pose(obj_pose)
        for config in traj:
            domain.set_config(config)
            if domain.world.is_body_pairwise_collision(
                body=robot, obstacles=[body]):
                return False
        return True
    return test
        

def postprocess(plan):
    paths = []
    if plan is None: return
    for name, args in plan:
        if name == 'pick':
            command = args[-1]
            paths += command[0]
            paths += [command[1]["grasp"]] #grasp
            paths += command[2]
        elif name == 'place':
            command = args[-1]
            paths += command[0]
            paths += [command[1]["placement"]] #grasp
            paths += command[2]
        elif name in ['move_free', 'move_holding']:
            paths += args[-1]
    return paths

def visualize(domain:TAMPDomain, mode_init:Mode, command_seq:List):
    mode = mode_init
    for command in command_seq:
        if isinstance(command, Config):
            domain.assign(mode, command)
        if isinstance(command, Attachment):
            mode.set_attachment(command)
        sleep(0.05)
        

if __name__ == "__main__":
    dom = DomainKitchen(gui=True, num_box=5)
    prob1 = ProblemKitchen(dom, num_block=5, cooked_list=[1, 2, 3, 4, 5])
    prob = pddlstream_from_problem(dom)
    
    solution = solve(
        prob, 
        algorithm="adaptive", 
        unit_costs=True
    )
    command = postprocess(solution.plan)
    visualize(dom, prob1.mode_init, command)

    input()