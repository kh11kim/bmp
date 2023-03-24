from numpy import choose
from bmp import *
from bmp.planner.rrt import BiRRT2
from pyperplan.task import Operator, Task
from pyperplan.planner import HEURISTICS, SEARCHES, _parse, _ground, _search
from pyperplan.pddl.parser import Parser
from typing import TypeVar
from dataclasses import dataclass
import time
import logging

T = TypeVar('T')

def choose_random(x:List[T])->T:
    i = np.random.randint(0, len(x))
    return x[i]

class QTable:
    def __init__(self, task):
        self.Q = {}
        self.visited = set()
        self.task = task
    
    def get_Qvalue(self, s, a):
        if s not in self.Q:
            self.Q[s] = {}
        if a not in self.Q[s]:
            self.Q[s][a] = 0
        return self.Q[s][a]
        
    def is_exist(self, s, a):
        if s in self.Q:
            if a in self.Q[s]:
                return True
        return False

    def argmax_Qvalue(self, s):
        actions = [op for op in self.task.operators if op.applicable(s)]
        values = [self.get_Qvalue(s, a) for a in actions]
        i = np.argmax(values)
        return actions[i]

    def update_tuple(self, s, a, r, s_new, alpha=0.1, gamma=0.99):
        a_new_argmax = self.argmax_Qvalue(s_new)
        if a_new_argmax is None:
            Q_new = 0
        else: 
            Q_new = self.get_Qvalue(s_new, a_new_argmax)
        Q_curr = self.get_Qvalue(s, a)
        tderr = (r + gamma*Q_new - Q_curr)
        Q_curr = Q_curr + alpha*tderr
        self.visited.update([s_new])

class TP:
    def __init__(self, domain_file, prob_file, planner="gbf", heuristic="hff"):
        self.domain_file = domain_file
        self.prob_file = prob_file
        self.planner = planner
        self.heuristic = heuristic
        self.parser = Parser(self.domain_file, probFile=self.prob_file)
        self.domain = self.parser.parse_domain()
        self.problem = self.parser.parse_problem(self.domain)
        self.task = _ground(self.problem)

        self.state_init = self.task.initial_state
        self.search = SEARCHES[self.planner]
        self.heuristic = HEURISTICS[self.heuristic](self.task)
        self.Q = QTable(self.task)

    def TP(self):
        eps = 0.2
        queue = [self.state_init]
        sampling_queue = []
        while len(queue) != 0:
            s = queue.pop()
            actions = [op for op in self.task.operators if op.applicable(s)]
            
            if np.random.rand() < eps:
                a = choose_random(actions)                
            else:
                a = self.Q.argmax_Qvalue(s)
            s_new = a.apply(s)
            sampling_queue.append((s, a, s_new))

            if s_new in self.Q.visited:
                queue.append(s_new)
        return sampling_queue
    

    def update(self, sampled_queue):
        for (s, a, r, s_new) in sampled_queue:
            self.Q.update_tuple(s, a, r, s_new)

class MMMP:
    def __init__(self, domain:TAMPDomain, s_init, sigma_init):
        self.domain = domain
        self.RT = {s_init:[sigma_init]}
    
    def get_attachment(self, a):
        sample_tuple = None
        for pred in a.del_effects:
            if not "attached" in pred:
                continue
            _, obj, parent = pred.strip("()").split(" ")
            sample_tuple = (obj, parent)
        if sample_tuple is None:
            return None
        else:
            (obj, parent) = sample_tuple
            return self.domain.sample_attachment(obj, parent)

    def sampling(self, sampling_queue):
        sampled_queue = []
        for (s, a, s_new) in sampling_queue:
            if s not in self.RT:
                continue
            sigma_parent: Mode = choose_random(self.RT[s])
            att = self.get_attachment(a)
            if att is None:
                r = 1
            else:
                sigma_new = sigma_parent.set_attachment(att)
            sampled_queue += [(s, a, r, s_new)]

            




if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)-8s %(message)s"
    )
    dom = DomainKitchen(gui=True, num_box=2)
    prob = ProblemKitchen(dom, num_block=2, cooked_list=[1, 2])
    domain_file = Path(__file__).parent.parent / "domain/kitchen/pddl/domain_new.pddl"
    prob_file = Path(__file__).parent.parent / "domain/kitchen/pddl/problem_box2.pddl"
    
    tp = TP(domain_file, prob_file)
    mmmp = MMMP(dom, tp.state_init, prob.mode_init)
    
    for i in range(10):
        sampling_queue = tp.TP()
        sampled_queue = mmmp.sampling(sampling_queue)
        tp.update(sampled_queue)
