;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; Kitchen domain : you have to wash a food before cook.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain KITCHEN_NEW)
	(:requirements :strips :typing)
	(:types
		robot placeable - object
		movable region - placeable
		sink oven dish - region
		food - movable
	)

	(:predicates 
		(attached ?obj - food ?parent - object)
		(handempty)
		(cleaned ?obj - food)
		(cooked ?obj - food)
		(ready)
	)
	
	(:action move-to-pick
		:parameters (?obj - food ?parent - region ?robot - robot)
		:precondition (and (attached ?obj ?parent) (handempty) (ready))
		:effect
			(and 
				(not (handempty))
				(attached ?obj ?robot)
				(not (ready))
			)
	)

	(:action pick
		:parameters (?obj - food ?parent - region ?robot - robot)
		:precondition (and (attached ?obj ?parent) (attached ?obj ?robot))
		:effect
			(and 
				(not (attached ?obj ?parent))
				(ready)
			)
	)

	(:action move-to-place
		:parameters (?obj - food ?robot - robot ?parent - region)
		:precondition (and (attached ?obj ?robot) (ready))
		:effect
		(and 
			(not (ready))
			(attached ?obj ?parent)
		)
	)

  	(:action place
		:parameters (?obj - food ?robot - robot ?parent - region)
		:precondition (and (attached ?obj ?robot))
		:effect
  		(and 
	     	(not (attached ?obj ?robot))
			(handempty)
			(ready)
		)     
	)

	(:action wash
	     :parameters (?obj - food ?parent - sink)
	     :precondition (and (attached ?obj ?parent))
	     :effect
	     (and (cleaned ?obj)))
  
  	(:action cook
	     :parameters (?obj - food ?parent - oven)
	     :precondition (and (attached ?obj ?parent) (cleaned ?obj))
	     :effect
	     (and (cooked ?obj))
	)
)
