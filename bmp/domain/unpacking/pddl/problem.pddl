(define 
  (problem PickAndPlace-1)
  (:domain PICK-AND-PLACE)
  (:objects
    robot - robot
    box1 box2 box3 box4 box5 box6 box7 box8 box9 - movable
    plate1 plate2 - region
  )
  (:init 
    (attached box1 plate2)
    (attached box2 plate2) 
    (attached box3 plate2)
    (attached box4 plate2)
    (attached box5 plate2)
    (attached box6 plate2)
    (attached box7 plate2)
    (attached box8 plate2)
    (attached box9 plate2)
    (clear box1)
    (clear box2)
    (clear box3)
    (clear box4)
    (clear box5)
    (clear box6)
    (clear box7)
    (clear box8)
    (clear box9)
    (handempty)
  )

  (:goal 
    (and 
      (attached box1 plate1)
    )
  )
)