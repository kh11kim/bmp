(define 
  (problem PickAndPlace-1)
  (:domain PICK-AND-PLACE)
  (:objects
    robot - robot
    red1 green1 blue1 obs1 obs2 obs3 - movable
    plate_red plate_green plate_blue - region
  )
  (:init 
    (attached red1 plate_green)
    (attached green1 plate_blue)
    (attached blue1 plate_red)
    (attached obs1 plate_green)
    (attached obs2 plate_blue)
    (attached obs3 plate_red)
    (handempty)
  )

  (:goal 
    (and 
      ;(attached red1 plate_red)
      (attached red1 plate_red)
      (attached green1 plate_green)
      (attached blue1 plate_blue)
      ; (attached blue1 plate_blue)
    ;   (attached blue1 plate_blue)
    ;   (attached blue2 plate_blue)
    )
  )
)