(define 
  (problem PickAndPlace-1)
  (:domain PICK-AND-PLACE)
  (:objects
    robot - robot
    dice1 dice2 - movable
    table - region
  )
  (:init 
    (attached dice1 table)
    (attached dice2 table)
    (handempty)
  )

  (:goal 
    (and 
      (attached dice1 table)
      (attached dice2 table)
    )
  )
)