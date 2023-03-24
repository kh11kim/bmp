from pyperplan.planner import HEURISTICS, SEARCHES, _parse, _ground, _search
from pyperplan.pddl.parser import Parser
from pyperplan.task import Task, Operator
from pathlib import Path
from dataclasses import dataclass, field
from typing import List, ClassVar
import numpy as np
from time import perf_counter


class MCTSNode:
    alpha = 0.6
    def __init__(self, state: frozenset, task: Task, is_root=False):
        self.state = state
        self.task = task
        self.children = {}
        self.num_visit = 0
        self.rewards = 0
        self.unexpanded_action: List[Operator] = self.get_unexpanded_action(self.state)
        self.is_root = is_root
    
    @staticmethod
    def is_nongeometric_action(action:Operator):
        non_geometric_actions = ["wash", "cook"]
        for action_name in non_geometric_actions:
            if action_name in action.name:
                return True
        return False


    def get_unexpanded_action(self, state:frozenset):
        actions = [op for op in self.task.operators if op.applicable(self.state)]
        actions = [a for a in actions if not self.is_nongeometric_action(a)]
        return actions

    @property
    def is_fully_expanded(self):
        return len(self.unexpanded_action) == 0

    def random_expansion(self):
        np.random.shuffle(self.unexpanded_action)
        action = self.unexpanded_action.pop()
        next_state = action.apply(self.state)
        child = MCTSNode(next_state, self.task)
        self.children[action] = child
        return child, action
    
    def expansion_by_action(self, action:Operator):
        if action in self.children.keys():
            return self.children[action]
        else:
            self.unexpanded_action.remove(action)
            next_state = action.apply(self.state)
            child = MCTSNode(next_state, self.task)
            self.children[action] = child
            return child

    def pw_test(self):
        left = self.num_visit**self.alpha
        right = (self.num_visit-1)**self.alpha
        #print(left, right)
        return np.floor(left) > np.floor(right)
    
    def add_visit(self):
        self.num_visit += 1
    
    def select_random_unexpanded_action(self):
        i = np.random.randint(0, len(self.unexpanded_action))
        action = self.unexpanded_action[i]
        return action

    def select_child_by_ucb(self):
        # apply ucb
        N = self.num_visit
        actions = list(self.children.keys())
        np.random.shuffle(actions) #tie breaker
        ucbs = []
        for action in actions:
            node: MCTSNode = self.children[action]
            ni = node.num_visit
            vi = node.rewards/node.num_visit
            ucb = vi + np.sqrt(2) * np.sqrt(np.log(N)/ni)
            ucbs += [ucb]
        i = np.argmax(ucbs)
        action = actions[i]
        return self.children[action], action

        # current_actions = list(self.children.keys())
        # i = np.random.randint(0, len(current_actions)) #random for now
        # action = current_actions[i]
        # return self.children[action], action

class PWUCT:
    def __init__(
        self, 
        domain_file:str,
        problem_file:str,
    ):
        self.domain_file = domain_file 
        self.problem_file = problem_file
        
        self.parser = Parser(
            self.domain_file, probFile=self.problem_file)
        self.domain = self.parser.parse_domain()
        self.problem = self.parser.parse_problem(self.domain)
        self.task = _ground(self.problem)

        self.search = SEARCHES["gbf"]
        heuristic_class = HEURISTICS["hadd"]
        self.heuristic = heuristic_class(self.task)

        self.root = MCTSNode(self.task.initial_state, self.task, is_root=True)
        
    def get_skeleton(self):
        action_seq = []
        node = self.root
        #selection
        while True:
            node.add_visit()
            if len(node.children) == 0: break
            elif (not node.is_fully_expanded) and (node.pw_test()): 
                break
            else: 
                node, action = node.select_child_by_ucb()
                action_seq += [action]
        
        #expansion
        node, action = node.random_expansion()
        node.add_visit()
        action_seq += [action]
        self.task.initial_state = node.state
        action_seq2 = _search(self.task, self.search, self.heuristic)
        for action in action_seq2:
            if MCTSNode.is_nongeometric_action(action):
                continue
            action_seq += [action]
        return action_seq
    
    def backpropagate(self, reward:float, actions):
        node = self.root
        for action in actions:
            node.rewards += reward
            if action not in node.children: break
            node = node.children[action]
            


