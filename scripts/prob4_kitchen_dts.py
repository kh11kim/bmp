from bmp import *
from pybullet_suite import *

from pyperplan.task import Operator, Task
from bmp.planner.tmts import *
import pandas as pd
from time import sleep

# class TMTS_pnp(TMTS):
#     def __init__(
#         self, 
#         domain_file, 
#         problem_file, 
#         mode_key,
#         nongeometric_actions,
#         domain:TAMPDomain,
#     ):
#         super().__init__(domain_file, problem_file, nongeometric_actions)
#         self.domain = domain

#     def visualize(self):
#         node_seq, traj_seq = self.postprocess()
#         for node, (mode, traj) in zip(node_seq, traj_seq):
#             for config in traj:
#                 self.domain.assign(mode, config)
#                 sleep(0.05)

#     def sample_goal(
#         self, v_init:Node, pi:List[Operator], max_iter=10
#     )->Node:
#         state = v_init.state
#         unchanged_preds = deepcopy(v_init.state)
#         for a in pi:
#             state = a.apply(state)
#             unchanged_preds = unchanged_preds.difference(a.del_effects)
#         changed_preds = state.difference(unchanged_preds)
        
#         v_goal = deepcopy(v_init)
#         v_goal.state = state
#         for _ in range(max_iter):
#             for pred in changed_preds:
#                 pred_name, *other = pred.strip("()").split(" ")
#                 if pred_name != "attached": continue
#                 obj, parent = other
#                 if obj in self.theta_goal: 
#                     v_goal.theta[obj] = deepcopy(self.theta_goal[obj])
#                 # sample
#                 if "robot" in parent:
#                     v_goal.theta[obj] = self.domain.sample_grasp(obj, parent)
#                 else:
#                     v_goal.theta[obj] = self.domain.sample_placement(obj, parent)
#             if not self.domain.is_collision(Mode(v_goal.theta), v_goal.q):
#                 return v_goal
#         return None

#     def sample_theta_add(self, v:Node, a:Operator) -> Attachment:
#         assert not self.is_nongeometric_action(a)
#         action_name, obj, p_del, p_add = a.name.strip("()").split(" ")
#         if action_name == "pick":
#             att = self.domain.sample_grasp(obj, p_add)
#         elif action_name == "place":
#             att = self.domain.sample_placement(obj, p_add)
#         return att

#     def sample_mode_param(self, v:Node, a:Operator)->Dict[str, Attachment]:
#         assert not self.is_nongeometric_action(a)
#         action_name, obj, p_del, p_add = a.name.strip("()").split(" ")
#         if action_name == "pick":
#             att = self.domain.sample_grasp(obj, p_add)
#         elif action_name == "place":
#             att = self.domain.sample_placement(obj, p_add)
#         theta_new = deepcopy(v.theta)
#         theta_new[obj] = att
#         return theta_new
    
#     def sample_transition(
#         self, v:Node, a:Operator, theta_new:Dict[str, Attachment]
#     )->Node:
#         assert not self.is_nongeometric_action(a)
#         mode_old = Mode(v.theta)
#         self.domain.assign(mode_old, v.q)

#         action_name, obj, p_del, p_add = a.name.strip("()").split(" ")
#         if action_name == "pick":
#             grasp = theta_new[obj]
#             placement = v.theta[obj]
#             placement_parent = p_del
#         else:
#             grasp = v.theta[obj]
#             placement = theta_new[obj]
#             placement_parent = p_add
        
#         parent_pose = self.domain.objects[placement_parent].get_base_pose()
#         obj_pose = parent_pose * placement.tf.inverse()
#         grasp_pose = obj_pose * grasp.tf
#         grasp_pose_pre = grasp.get_pre_pose(grasp_pose)
        
#         robot = self.domain.robots["robot"]
#         q_pre_ik = robot.inverse_kinematics(pose=grasp_pose_pre)
#         if q_pre_ik is None: return None
#         config_pre = deepcopy(v.q)
#         config_pre.set_q("robot", q_pre_ik)
#         mode_new = Mode(theta_new)
#         if self.domain.is_collision(mode_new, config_pre): 
#             return None
        
#         mp = BiRRT2()
#         traj_switch = mp.check_mode_switch(
#             config_pre.robot_names, config_pre, grasp_pose, mode_old, self.domain)
#         if traj_switch is not None: 
#             return Node(config_pre, theta_new, traj_switch=traj_switch)
#         return None

#     def get_single_mode_path(self, phi:Dict[str,str], v:Node, v_new:Config):
#         if v_new is None: return None
#         mode_old = Mode(v.theta)
#         mp = BiRRT2(q_delta_max=0.1, ts_eps=0.01)
#         traj = mp.plan(v.q.robot_names, v.q, v_new.q, mode_old, self.domain)
#         if traj is not None: 
#             return traj
#         return None
    
#     def extend(self, v_parent:Node, a:Operator, theta_add:Attachment):
#         action_name, obj, phi_del, phi_add = a.name.strip("()").split(" ")
#         state_new = a.apply(v_parent.state)
#         theta_new = deepcopy(v_parent.theta)
#         theta_new[obj] = theta_add
#         v_new = self.sample_transition(v_parent, a, theta_new)
#         tau = self.get_single_mode_path("not_needed", v_parent, v_new)
#         if tau is not None:
#             self.add_mode_node(v_new, state_new, v_parent)
#             self.add_edge(v_parent, v_new, tau)
#             return v_new
#         return None



if __name__ == "__main__":
    dom = DomainKitchen(gui=True, num_box=5)
    prob = ProblemKitchen(dom, num_block=5, cooked_list=[1, 2, 3, 4, 5])
    domain_file = Path(__file__).parent.parent / "bmp/domain/kitchen/pddl/domain.pddl"
    problem_file = Path(__file__).parent.parent / "bmp/domain/kitchen/pddl/problem_ex.pddl"
    
    Config.robot_names = ["robot"]
    mode_key = ["box1", "box2", "box3", "box4", "box5"]
    nongeometric_actions = ["wash", "cook"]
    q = prob.config_init
    theta = prob.mode_init.attachments
    
    np.random.seed(1)
    df = pd.DataFrame()
    test_conds = [
        # ("bi", 0., 0.1, "prop_task0_reach0.1"),
        # ("bi", 0., 0.3, "prop_task0_reach0.3"),
        # ("bi", 0., 0.5, "prop_task0_reach0.5"),
        # ("bi", 0.2, 0.1, "prop_task0.2_reach0.1"),
        # ("bi", 0.2, 0.3, "prop_task0.2_reach0.3"),
        # ("bi", 0.2, 0.5, "prop_task0.2_reach0.5"),
        # ("bi", 0.4, 0.1, "prop_task0.4_reach0.1"),
        # ("bi", 0.4, 0.3, "prop_task0.4_reach0.3"),
        ("bi", 0.4, 0.3, "prop_task0.4_reach0.3"),
    ]
    for i, test_cond in enumerate(test_conds, start=1):
        algo, eps_task, eps_reach, label = test_cond
        elapsed = []
        print(f"=={label}==")
        for i in range(1):
            start = time.perf_counter()
            tmts = TMTS_pnp(
                domain_file, problem_file, mode_key=mode_key, 
                nongeometric_actions=nongeometric_actions,
                domain=dom)
            if algo == "fwd":
                tmts.plan_fwd(q, theta, eps_task=eps_task, alpha=0.8)
            else:
                tmts.plan_bi(q, theta, eps_task=eps_task, eps_reach=eps_reach, alpha=0.8)
            end = time.perf_counter()
            elapsed += [end-start]
            tmts.visualize()
            print(f"iter: {i} elapsed: {end-start}")
        df[label] = elapsed
        print(f"avg: {np.mean(elapsed)}")
    df.to_csv("test_result.csv")
    

