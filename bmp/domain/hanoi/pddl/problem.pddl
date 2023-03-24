(define 
(problem hanoi_ex)
(:domain HANOI)
(:objects
    robot - robot
    peg1 peg2 peg3 - peg
    disk1 disk2 disk3 - disk
)
(:init 
    (clear disk1) 
    (clear peg2) 
    (clear peg3)
    (smaller disk1 disk2)
    (smaller disk1 disk3)
    (smaller disk2 disk3)
    (smaller disk1 peg1)
    (smaller disk1 peg2)
    (smaller disk1 peg3)
    (smaller disk2 peg1)
    (smaller disk2 peg2)
    (smaller disk2 peg3)
    (smaller disk3 peg1)
    (smaller disk3 peg2)
    (smaller disk3 peg3)
    (attached disk3 peg1) 
    (attached disk2 disk3) 
    (attached disk1 disk2)
    (handempty))

(:goal (and 
    (attached disk3 peg3) 
    (attached disk2 disk3) 
    (attached disk1 disk2)    
    (handempty)
    )))