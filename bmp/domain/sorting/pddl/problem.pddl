(define 
  (problem PickAndPlace-1)
  (:domain PICK-AND-PLACE)
  (:objects
    robot - robot
    red1 red2 red3 red4 red5 green1 green2 green3 green4 green5 obs1 obs2 obs3 obs4 obs5 obs6 obs7 obs8 obs9 obs10 - movable ;red7 red8 red9 red10 obs7 obs8 obs9 obs10 green7 green8 green9 green10 
    table_red table_green table_gray1 - region ;table_gray2
  )
  (:init 
    (attached red1 table_gray1)
    (attached red2 table_gray1)
    (attached red3 table_gray1)
    (attached red4 table_gray1)
    (attached red5 table_gray1)
    ; (attached red6 table_gray2)
    ; (attached red7 table_gray2)
    ; (attached red8 table_gray2)
    (attached green1 table_gray1)
    (attached green2 table_gray1)
    (attached green3 table_gray1)
    (attached green4 table_gray1)
    (attached green5 table_gray1)
    ; (attached green6 table_gray2)
    ; (attached green7 table_gray2)
    ; (attached green8 table_gray2)
    (attached obs1 table_gray1)
    (attached obs2 table_gray1)
    (attached obs3 table_gray1)
    (attached obs4 table_gray1)
    (attached obs5 table_gray1)
    (attached obs6 table_gray1)
    (attached obs7 table_gray1)
    (attached obs8 table_gray1)
    (attached obs9 table_gray1)
    (attached obs10 table_gray1)
    ; (attached obs11 table_gray1)
    ; (attached obs12 table_gray1)
    ; (attached obs6 table_gray2)
    ; (attached obs7 table_gray2)
    ; (attached obs8 table_gray2)

    ; (attached red7 table_gray2)
    ; (attached red8 table_gray2)
    ; (attached red9 table_gray2)
    ; (attached red10 table_gray2)
    ; (attached green7 table_gray2)
    ; (attached green8 table_gray2)
    ; (attached green9 table_gray2)
    ; (attached green10 table_gray2)
    (handempty)
  )

  (:goal 
    (and 
      (attached red1 table_red)
      (attached red2 table_red)
      (attached red3 table_red)
      (attached red4 table_red)
      (attached red5 table_red)
      ; (attached red6 table_red)
      ; (attached red7 table_red)
      ; (attached red8 table_red)
      (attached green1 table_green)
      (attached green2 table_green)
      (attached green3 table_green)
      (attached green4 table_green)
      (attached green5 table_green)

      (attached obs1 table_gray1)
      (attached obs2 table_gray1)
      (attached obs3 table_gray1)
      (attached obs4 table_gray1)
      (attached obs5 table_gray1)
      (attached obs6 table_gray1)
      (attached obs7 table_gray1)
      (attached obs8 table_gray1)
      (attached obs9 table_gray1)
      (attached obs10 table_gray1)
      ; (attached green6 table_green)
      ; (attached green7 table_green)
      ; (attached green8 table_green)
      ; (attached red7 table_red)
      ; (attached red8 table_red)
      ; (attached red9 table_red)
      ; (attached red10 table_red)
      ; (attached green7 table_green)
      ; (attached green8 table_green)
      ; (attached green9 table_green)
      ; (attached green10 table_green)
    )
  )
)