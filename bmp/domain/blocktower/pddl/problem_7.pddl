(define 
  (problem PickAndPlace-1)
  (:domain PICK-AND-PLACE)
  (:objects
    robot - robot
    block1 block2 block3 block4 block5 block6 block7 - movable
    plate1 plate2 plate3 - region
  )
  (:init 
    (attached block1 block2)
    (attached block2 block3)
    (attached block3 plate3)
    (attached block4 block5)
    (attached block5 block6)
    (attached block6 block7)
    (attached block7 plate2)
    (clear block1)
    (clear block4)
    (handempty)
  )

  (:goal 
    (and 
      (attached block1 block2)
      (attached block2 block3)
      (attached block3 block4)
      (attached block4 block5)
      (attached block5 block6)
      (attached block6 block7)
      (attached block7 plate1)
    )
  )
)