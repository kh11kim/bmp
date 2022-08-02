(define 
(problem KITCHEN-4-0)
(:domain KITCHEN)
(:objects
    box1 box2 box3 box4 - food
    sink1 - sink
    dish1 - dish
    oven1 - oven
)
(:init (clear box1) (clear box2) (clear box3) (clear box4) 
    (on dish1 box1) (on dish1 box2) (on dish1 box3) (on dish1 box4)
    (handempty))
(:goal (and (cooked box1) (cooked box2) (cooked box3) (cooked box4)))
)