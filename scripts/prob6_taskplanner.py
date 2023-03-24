from pyperplan.planner import HEURISTICS, SEARCHES, _parse, _ground, _search
from pyperplan.pddl.parser import Parser
from pyperplan.task import Task, Operator
from pathlib import Path
from dataclasses import dataclass, field
from typing import List, ClassVar
import numpy as np
from time import perf_counter


class MCTSNode:
    alpha = 0.5
    def __init__(self, state: frozenset, task: Task, is_root=False):
        self.state = state
        self.task = task
        self.children = {}
        self.num_visit = 0
        self.unexpanded_action: List[Operator] = [op for op in task.operators if op.applicable(self.state)]
        self.is_root = is_root
    
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
        print(left, right)
        return np.floor(left) > np.floor(right)
    
    def add_visit(self):
        self.num_visit += 1
    
    def select_random_unexpanded_action(self):
        i = np.random.randint(0, len(self.unexpanded_action))
        action = self.unexpanded_action[i]
        return action

    def select_child_by_ucb(self):
        # apply ucb
        current_actions = list(self.children.keys())
        i = np.random.randint(0, len(current_actions)) #random for now
        action = current_actions[i]
        return self.children[action], action

def main():
    domain_file = Path(__file__).parent.parent / "bmp/domain/kitchen/pddl/domain.pddl"
    problem_file = Path(__file__).parent.parent / "bmp/domain/kitchen/pddl/problem_ex.pddl"
    search = SEARCHES["wastar"]
    heuristic_class = HEURISTICS["hadd"]
    
    parser = Parser(domain_file.as_posix(), probFile=problem_file.as_posix())
    domain = parser.parse_domain()
    problem = parser.parse_problem(domain)
    task = _ground(problem)
    heuristic = heuristic_class(task)
    
    #start
    root = MCTSNode(task.initial_state, task, is_root=True)
    node = root
    for _ in range(100):
        action_seq = []
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
        action_seq += [action]

        task.initial_state = node.state
        
        start = perf_counter()
        action_seq += _search(task, search, heuristic)
        end = perf_counter()
        print(end-start)
        # simulation : do plan
        #task.initial_state = node_new.state
        
        
        # backpropagate
        node = root
    

    print("end")

if __name__ == "__main__":
    main()