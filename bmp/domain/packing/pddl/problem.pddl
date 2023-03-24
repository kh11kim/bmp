(define 
  (problem PickAndPlace-1)
  (:domain PICK-AND-PLACE)
  (:objects
    robot - robot
    box1 box2 box3 box4 box5  - movable
    plate1 plate2 - region
  )
  (:init 
    (attached box1 plate2)
    (attached box2 plate2) 
    (attached box3 plate2)
    (attached box4 plate2)
    (attached box5 plate2)
    (clear box4)
    (clear box3)
    (clear box2)
    (clear box1)
    (clear box5)
    (handempty)
  )

  (:goal 
    (and 
      (attached box1 plate1)
      (attached box2 plate1)
      (attached box3 plate1)
      (attached box4 plate1)
      (attached box5 plate1)
    )
  )
)