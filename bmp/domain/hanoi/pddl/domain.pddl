;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; The tower of HANOI domain : you have to move small disc to move bigger one below.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain HANOI)
  (:requirements :strips :typing)
  (:types
    robot placeable - object
  	movable region - placeable
    peg - region
    disk - movable
  )

  (:predicates 
    (attached ?obj - movable ?parent - object)
    (handempty)
    (clear ?obj - placeable)
    (smaller ?obj1 - movable ?obj2 - object)
  )

  (:action pick
	     :parameters (?obj - movable ?parent - placeable ?robot - robot)
	     :precondition (and (attached ?obj ?parent) (clear ?obj) (handempty))
	     :effect
	     (and 
	      (not (attached ?obj ?parent))
        (attached ?obj ?robot)
	      (not (handempty))
        (clear ?parent)))

  (:action place
	     :parameters (?obj - movable ?robot - robot ?parent - placeable)
	     :precondition (and (attached ?obj ?robot) (smaller ?obj ?parent) (clear ?parent))
	     :effect
	     (and 
	      (not (attached ?obj ?robot))
        (attached ?obj ?parent)
	      (handempty)
        (not (clear ?parent))
        )))
