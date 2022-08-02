;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Kitchen domain : you have to wash a food before cook.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain KITCHEN)
  (:requirements :strips :typing)
  (:types
    robot movable region - object
    sink oven dish - region
    food - movable
  )

  (:predicates 
    (on ?parent - region ?obj - food)
    (handempty)
    (holding ?obj - food)
    (clear ?obj - food)
    (cleaned ?obj - food)
    (cooked ?obj - food)
  )

  (:action wash
	     :parameters (?obj - food ?parent - sink)
	     :precondition (and (on ?parent ?obj))
	     :effect
	     (and (cleaned ?obj)))
  
  (:action cook
	     :parameters (?obj - food ?parent - oven)
	     :precondition (and (on ?parent ?obj) (cleaned ?obj))
	     :effect
	     (and (cooked ?obj)))

  (:action pick
	     :parameters (?obj - food ?parent - region)
	     :precondition (and (clear ?obj) (on ?parent ?obj) (handempty))
	     :effect
	     (and (not (on ?parent ?obj))
		   (not (clear ?obj))
		   (not (handempty))
		   (holding ?obj)))

  (:action place
	     :parameters (?obj - food ?parent - region)
	     :precondition (holding ?obj)
	     :effect
	     (and (not (holding ?obj))
		   (clear ?obj)
		   (handempty)
		   (on ?parent ?obj))))

  ; (:action stack
	;      :parameters (?x ?y)
	;      :precondition (and (holding ?x) (clear ?y))
	;      :effect
	;      (and (not (holding ?x))
	; 	   (not (clear ?y))
	; 	   (clear ?x)
	; 	   (handempty)
	; 	   (on ?x ?y)))
  ; (:action unstack
	;      :parameters (?x ?y)
	;      :precondition (and (on ?x ?y) (clear ?x) (handempty))
	;      :effect
	;      (and (holding ?x)
	; 	   (clear ?y)
	; 	   (not (clear ?x))
	; 	   (not (handempty))
	; 	   (not (on ?x ?y)))))