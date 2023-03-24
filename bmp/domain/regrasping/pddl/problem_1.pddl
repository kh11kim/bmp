(define 
  (problem PickAndPlace-1)
  (:domain PICK-AND-PLACE)
  (:objects
    robot - robot
    dice1 - movable ;red7 red8 red9 red10 obs7 obs8 obs9 obs10 green7 green8 green9 green10 
    table - region ;table_gray2
  )
  (:init 
    (attached dice1 table)
    (handempty)
  )

  (:goal 
    (and 
      (attached dice1 table)
    )
  )
)