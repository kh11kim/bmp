from bmp.domain import *
from bmp.planner.rrt import BiRRT2
from pddlstream.language.constants import PDDLProblem, print_solution
from pddlstream.utils import read, get_file_path
from pddlstream.algorithms.meta import solve
from pddlstream.language.generator import from_fn, from_gen_fn, from_test

@dataclass
class Trajectory:
    traj: List
    def __repr__(self):
        return f"Traj"

def pddlstream_from_problem(domain_pddl_path, stream_pddl_path, prob:ProblemBlocktower, num_block:int)->PDDLProblem:
    domain = prob.domain

    domain_pddl = read(domain_pddl_path)
    stream_pddl = read(stream_pddl_path)
    constant_map = {}
    
    panda = domain.robots["robot"]
    config_init = prob.config_init
    movables = list(domain.movables.values())
    regions = list(domain.regions.values())
    init = [
        ("Arm", panda),
        ("AConf", config_init),
        ("AtAConf", panda, config_init),
        ("HandEmpty", panda),
        #("CanMove",),
    ]

    mode = prob.mode_init
    region_poses = {}
    for region_name, region in domain.regions.items():
        init += [('Plate', region)]

    for movable in movables:
        init += [
            ("Block", movable),
        ]
        # for region in domain.regions.values():
        #     if prob.is_placed_peg(movable, region):
        #         sop = movable.sops[0]
        #         placement = domain.get_current_placement(movable.name, region.name, sop)
        #         placements[movable.name] = placement
        if movable.name in mode.attachments:
            att = mode.attachments[movable.name]
            init += [('Pose', movable, att)]
            init += [('AtPose', movable, att)]
            parent = domain.objects[att.parent_name]
            if att.parent_name in mode.attachments:    
                att2 = mode.attachments[parent.name]
                init += [('Block-supported', movable, att, parent, att2)] #, 
                init += [('On-Block', movable, parent)]
            else:
                init += [('Plate-supported', movable, att, parent)] #, 
                init += [('On-plate', movable, parent)]
        
    # for movable in movables:
    #     for movable2 in movables:
    #         if prob.is_placed_disc(movable, movable2):
    #             sop = movable.sops[0]
    #             placement = domain.get_current_placement(movable.name, movable2.name, sop)
    #             placement2 = placements[movable2.name]
    #             init += [('Pose', movable, placement)]
    #             init += [('AtPose', movable, placement)]
    #             init += [('Disc-upported', movable, placement, movable2, placement2)] #, 
    #             init += [('On-disc', movable, movable2)]

    movables = domain.movables
    regions = domain.regions
    
    if num_block == 4:
        clear_block = [1, 3]
        goal_order = [1, 2, 3, 4]
    elif num_block == 5:
        clear_block = [1, 3]
        goal_order = [1, 2, 3, 4, 5]
    elif num_block == 6:
        clear_block = [1, 4]
        goal_order = [1, 2, 3, 4, 5, 6]

    for i in clear_block:
        init += [("Clear", movables[f'block{i}'])]

    goal = ["and", ("AtAConf", panda, config_init),]
    for i in goal_order[:-1]:
        next_block = movables[f"block{i+1}"]
        goal += [("On-block", movables[f"block{i}"], next_block)]
    goal += [("On-plate", next_block, regions['plate1'])]

    
    stream_map = {
        'sample-plate-place-pose': from_gen_fn(get_stable_gen(domain)),
        'sample-block-place-pose': from_gen_fn(get_stable_gen2(domain)),
        'sample-grasp': from_gen_fn(get_grasp_gen(domain)),
        'inverse-kinematics': from_gen_fn(get_ik_ir_gen(domain, prob.mode_init)),
        'plan-arm-motion': from_fn(get_arm_motion_fn(domain, teleport=False)),
        
        #'plan-approach-motion': from_fn(get_approach_motion_fn(domain, teleport=False)),
        'test-cfree-pose-pose': from_test(get_cfree_pose_pose_test(domain)),
        'test-cfree-approach-pose': from_test(get_cfree_approach_pose_test(domain)),
        'test-cfree-traj-pose': from_test(get_cfree_traj_pose_test(domain)),
        # 'test-kin': from_test(get_test_kin(domain)),

    }
    
    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

def get_stable_gen(domain:TAMPDomain):
    def gen(body:Movable, support:TAMPObject): #, base_pose
        while True:
            placement = domain.sample_placement(body.name, support.name)
            yield (placement,)
    return gen
    
def get_stable_gen2(domain:TAMPDomain):
    def gen(body:Movable, support_body:Movable, support_placement:Placement): #, base_pose
        while True:
            support_body.set_base_pose(support_placement.obj_pose)
            #body.set_base_pose()
            placement = domain.sample_placement(body.name, support_body.name)
            body.set_base_pose(placement.obj_pose)
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
            if parent_name in domain.objects:
                #parent_pose = placement.parent_tf
                #domain.objects[parent_name].get_base_pose()
                #obj_pose = parent_pose * placement.tf.inverse()
                obj_pose = placement.obj_pose
                body.set_base_pose(obj_pose)
                grasp_pose = obj_pose * grasp.tf
                grasp_pre_pose = grasp.get_pre_pose(grasp_pose)
                q = robot.inverse_kinematics(pose=grasp_pose)
                domain.set_config(Config({"robot":q}))
                if q is None: return None
                q_pre = robot.inverse_kinematics(pose=grasp_pre_pose)
                if q_pre is None: return None
                else: q_pre = Config(q={robot_name:q_pre})
                
                mp = BiRRT2()
                path = mp.check_mode_switch(
                    [robot_name], q_pre, grasp_pose, mode, domain, col_check=False) #mode is ignored
                if path is None: return None
                yield (q_pre, Trajectory(path))
    return gen

def get_cfree_pose_pose_test(domain:TAMPDomain):
    def test(body:Body, placement:Placement, body2:Body, placement2:Placement):
        if body == body2:
            return False
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
        # parent_name = placement.parent_name
        # parent_pose = domain.regions[parent_name].get_base_pose()
        obj_pose = placement.obj_pose #parent_pose * placement.tf.inverse()
        body.set_base_pose(obj_pose)
        for config in traj.traj:
            domain.set_config(config)
            if domain.world.is_body_pairwise_collision(
                body=robot, obstacles=[body]):
                return False
        return True
    return test

def get_arm_motion_fn(domain:TAMPDomain, teleport=False):
    robot_name = list(domain.robots.keys())[0]
    def fn(robot: Panda, conf1: Config, conf2: Config, fluents=[]):
        if teleport:
            path = [conf1, conf2]
        else:
            atts = {}
            parents = {}
            for fluent in fluents:
                if fluent[0] == "atpose":
                    movable:Movable = fluent[1]
                    att:Placement = fluent[2]
                    movable.set_base_pose(att.obj_pose)
                    parents[movable.name] = att.parent_name
                elif fluent[0] == "atgrasp":
                    movable:Movable = fluent[2]
                    att:Grasp = fluent[3]
                    atts[movable.name] = att
                    parents[movable.name] = att.parent_name
            mode = Mode(atts)
                
            #planning in mode
            mp = BiRRT2()
            path = mp.plan([robot_name], conf1, conf2, mode, domain) #only fixed setting

            if path is None: return None
            return (Trajectory(path),)
    return fn

def pddlstream_solve(prob):
    planner = "ff-astar"
    solution = solve(
        prob, 
        algorithm="adaptive", 
        planner=planner,
        max_time=100,
        
        verbose=True, 
        debug=False,
        unit_efforts=True,
    )
    print_solution(solution)
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
        elif a.name == "pick" or a.name == "place":
            #(?arm ?disc ?peg ?p ?g ?q ?t)
            if a.name == "pick":
                att = a.args[4]
            elif a.name == "place":
                att = a.args[3]
            traj_approach = a.args[-1].traj
            for config in traj_approach:
                domain.assign(mode, config)
                time.sleep(dt)
            mode.set_attachment(att)
            for config in traj_approach[::-1]:
                domain.assign(mode, config)
                time.sleep(dt)
        elif a.name == "stack" or a.name == "unstack":
            #(?arm ?disc ?lowerdisc ?p ?g ?lowerp ?q ?t)
            if a.name == "stack":
                att = a.args[3]
            elif a.name == "unstack":
                att = a.args[4]
            traj_approach = a.args[-1].traj
            for config in traj_approach:
                domain.assign(mode, config)
                time.sleep(dt)
            mode.set_attachment(att)
            for config in traj_approach[::-1]:
                domain.assign(mode, config)
                time.sleep(dt)
