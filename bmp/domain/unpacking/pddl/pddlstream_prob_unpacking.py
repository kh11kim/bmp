from bmp.domain import *
from bmp.planner.rrt import BiRRT2
from pddlstream.language.constants import PDDLProblem
from pddlstream.utils import read, get_file_path
from pddlstream.algorithms.meta import solve
from pddlstream.language.generator import from_fn, from_gen_fn, from_test

@dataclass
class Trajectory:
    traj: List
    def __repr__(self):
        return f"Traj"

def pddlstream_from_problem(domain_pddl_path, stream_pddl_path, prob:ProblemHanoi)->PDDLProblem:
    domain = prob.domain

    domain_pddl = read(domain_pddl_path)
    stream_pddl = read(stream_pddl_path)
    constant_map = {}
    
    panda = domain.robots["robot"]
    config_init = prob.config_init
    movables = list(domain.movables.values())
    init = [
        ("Arm", panda),
        ("AConf", config_init),
        ("AtAConf", panda, config_init),
        ("HandEmpty", panda),
        ("CanMove",),
    ]

    for movable in movables:
        init += [
            ("Graspable", movable),
        ]
        for region in domain.regions.values():
            init += [
                ('Stackable', movable, region)
            ]
            if prob.is_placed(movable, region):
                sop = movable.sops[0]
                placement = domain.get_current_placement(movable.name, region.name, sop)
                init += [('Pose', movable, placement)]
                init += [('AtPose', movable, placement)]
                init += [('Supported', movable, placement, region)]
    
    init += [
        ('Smaller', movables["peg1"], movables["peg1"]),

    ]
    # for name, region in domain.regions.items():
    #     if "sink" in name:
    #         init += [("Sink", region)]
    #     if "oven" in name:
    #         init += [("Stove", region)]

    goal = [
        "and",
        ("AtAConf", panda, config_init),
    ]

    goal_movable = domain.movables["box1"]
    goal_placeable = domain.regions["plate1"]
    goal = (
        "and",
        ("AtAConf", panda, config_init),
        ("On", goal_movable, goal_placeable),
    )
    
    stream_map = {
        'sample-pose': from_gen_fn(get_stable_gen(domain)),
        'sample-grasp': from_gen_fn(get_grasp_gen(domain)),
        'inverse-kinematics': from_gen_fn(get_ik_ir_gen(domain, prob.mode_init)),
        'plan-arm-motion': from_fn(get_arm_motion_fn(domain, teleport=False)),

        'test-cfree-pose-pose': from_test(get_cfree_pose_pose_test(domain)),
        'test-cfree-approach-pose': from_test(get_cfree_approach_pose_test(domain)),
        'test-cfree-traj-pose': from_test(get_cfree_traj_pose_test(domain)),

    }
    
    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

def get_stable_gen(domain:TAMPDomain):
    def gen(body:Movable, support:TAMPObject):
        while True:
            placement = domain.sample_placement(body.name, support.name)
            yield (placement,)
    return gen

def get_grasp_gen(domain:TAMPDomain):
    robot_name = list(domain.robots.keys())[0]
    def gen(body:Movable):
        while True:
            grasp = domain.sample_grasp(body.name, robot_name)
            yield (grasp,)
    return gen

def get_ik_ir_gen(domain:TAMPDomain, mode):
    robot_name = list(domain.robots.keys())[0]
    robot: Panda = domain.robots[robot_name]
    def gen(robot:Panda, body:Movable, placement:Placement, grasp:Grasp):
        while True:
            parent_name = placement.parent_name
            if parent_name in domain.regions:
                parent_pose = domain.objects[parent_name].get_base_pose()
                obj_pose = parent_pose * placement.tf.inverse()
                grasp_pose = obj_pose * grasp.tf
                grasp_pre_pose = grasp.get_pre_pose(grasp_pose)
                q = robot.inverse_kinematics(pose=grasp_pose)
                if q is None: return None
                q_pre = robot.inverse_kinematics(pose=grasp_pre_pose)
                if q_pre is None: return None
                else: q_pre = Config(q={robot_name:q_pre})
                
                mp = BiRRT2()
                path = mp.check_mode_switch(
                    [robot_name], q_pre, grasp_pose, mode, domain, col_check=False) #mode is ignored

                yield (q_pre, Trajectory(path))
    return gen

def get_arm_motion_fn(domain:TAMPDomain, teleport=False):
    robot_name = list(domain.robots.keys())[0]
    def fn(robot: Panda, conf1: Config, conf2: Config, fluents=[]):
        if teleport:
            path = [conf1, conf2]
        else:
            atts = {}
            for fluent in fluents:
                if fluent[0] == "AtPose":
                    movable:Movable = fluent[1]
                    atts[movable.name] = fluent[2]
                elif fluent[0] == "AtGrasp":
                    movable:Movable = fluent[1]
                    atts[movable.name] = fluent[2]
            mode = Mode(atts)
            #planning in mode
            mp = BiRRT2()
            path = mp.plan([robot_name], conf1, conf2, mode, domain, only_fixed=True)

            if path is None: return None
        return (Trajectory(path),)
    return fn

def get_cfree_pose_pose_test(domain:TAMPDomain):
    def test(body:Body, placement:Placement, body2:Body, placement2:Placement):
        if body == body2:
            return True
        return not domain.is_collision_between_two_placement(placement, placement2)
    return test

def get_cfree_approach_pose_test(domain:TAMPDomain):
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
    robot = domain.robots["robot"]
    def test(
        traj:List[Config], body:Body, placement:Placement
    ):
        parent_name = placement.parent_name
        parent_pose = domain.regions[parent_name].get_base_pose()
        obj_pose = parent_pose * placement.tf.inverse()
        body.set_base_pose(obj_pose)
        for config in traj.traj:
            domain.set_config(config)
            if domain.world.is_body_pairwise_collision(
                body=robot, obstacles=[body]):
                return False
        return True
    return test

def pddlstream_solve(prob):
    planner = "ff-astar"
    solution = solve(
        prob, 
        algorithm="adaptive", 
        planner=planner,
        max_time=300,
        verbose=True, 
        debug=False,
        unit_efforts=True,
    )
    return solution

def visualize(prob:ProblemKitchen, solution):
    dt = 0.05
    plan = solution.plan
    if plan is None: return
    domain = prob.domain
    mode = prob.mode_init
    robot = domain.robots["robot"]
    for a in plan:
        if a.name == "move_arm":
            traj = a.args[-1].traj
            for config in traj:
                domain.assign(mode, config)
                time.sleep(dt)
        if a.name == "pick" or a.name == "place":
            if a.name == "pick":
                att = a.args[3]
            if a.name == "place":
                att = a.args[2]
            traj_approach = a.args[-1].traj
            for config in traj_approach:
                domain.assign(mode, config)
                time.sleep(dt)
            mode.set_attachment(att)
            for config in traj_approach[::-1]:
                domain.assign(mode, config)
                time.sleep(dt)