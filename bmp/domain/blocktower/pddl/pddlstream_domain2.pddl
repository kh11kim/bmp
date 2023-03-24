(define (domain pr2-tamp)
  (:requirements :strips :equality :disjunctive-preconditions :negative-preconditions :derived-predicates)
  ;(:constants @sink @stove)
  (:predicates
    ;static predicates
    (Arm ?arm)
    (Block ?block)
    (Plate ?plate)
    (Pose ?block ?p)
    (Grasp ?block ?g)
    (ApproachMotion ?t)
    (AConf ?q)
    (ATraj ?t)

    (Kin ?arm ?block ?p ?g ?q ?t)
    (ArmMotion ?arm ?q1 ?t ?q2)
    (Plate-supported ?block ?p ?plate)
    (Block-supported ?block ?p ?lowerblock ?lowerp)

    ;fluents
    (HandEmpty ?arm)
    (AtAConf ?arm ?q)
    (AtPose ?block ?p)
    (AtGrasp ?arm ?block ?g)
    (Clear ?block)
    (On-plate ?block ?plate)
    (On-block ?block ?lowerblock)

    ;test
    (CFreePosePose ?block ?p ?block2 ?p2)
    (CFreeApproachPose ?block ?p ?g ?block2 ?p2)
    (CFreeTrajPose ?t ?block2 ?p2)
    

    ;derived
    (UnsafePose ?block ?p)
    (UnsafeApproach ?block ?p ?g)
    (UnsafeATraj ?t)
  )
    (:derived (UnsafePose ?block ?p)
    (exists (?block2 ?p2) (and (Pose ?block ?p) (Pose ?block2 ?p2) (not (= ?block ?block2))
                           (not (CFreePosePose ?block ?p ?block2 ?p2))
                           (AtPose ?block2 ?p2)))
    )
    (:derived (UnsafeApproach ?block ?p ?g)
        (exists (?block2 ?p2) 
            (and 
                (Pose ?block ?p) 
                (Grasp ?block ?g) 
                (Pose ?block2 ?p2) 
                (not (= ?block ?block2))
                (not (CFreeApproachPose ?block ?p ?g ?block2 ?p2))
                (AtPose ?block2 ?p2)
            ))
    )
    (:derived (UnsafeATraj ?t)
        (exists (?block2 ?p2) 
            (and 
                (ATraj ?t)
                (Pose ?block2 ?p2)
                (not (CFreeTrajPose ?t ?block2 ?p2))
                (AtPose ?block2 ?p2)))
    )

    (:action pick
        :parameters (?arm ?block ?plate ?p ?g ?q ?t) ; ?rp 
        :precondition (and 
            ;type
            (Arm ?arm)
            (Block ?block)
            (Plate ?plate)
            (Pose ?block ?p)
            (Grasp ?block ?g)
            (AConf ?q)
            (ApproachMotion ?t)
            
            ;fluent-static
            (Kin ?arm ?block ?p ?g ?q ?t)
            (Clear ?block)
            (AtAConf ?arm ?q)
            (not (UnsafeApproach ?block ?p ?g))
            
            ;fluent-effect
            (AtPose ?block ?p)
            (HandEmpty ?arm)
            (On-plate ?block ?plate)
        )
        :effect (and 
            (not (AtPose ?block ?p)) 
            (AtGrasp ?arm ?block ?g)
            (not (HandEmpty ?arm))
            (not (On-plate ?block ?plate))
        )

    )
    (:action move_arm
        :parameters (?arm ?q1 ?q2 ?t)
        :precondition (and
                (ArmMotion ?arm ?q1 ?t ?q2)
                (AtAConf ?arm ?q1)
                (not (UnsafeATraj ?t))
            )
        :effect (and 
                (AtAConf ?arm ?q2)
                (not (AtAConf ?arm ?q1))
            )
    )

  
    (:action place
    :parameters (?arm ?block ?plate ?p ?g ?q ?t) ; ?rp 
    :precondition (and 
            ;type
            (Arm ?arm)
            (Block ?block)
            (Plate ?plate)
            (Pose ?block ?p)
            (Grasp ?block ?g)
            (AConf ?q)
            (ApproachMotion ?t)
            ;condition
            (Kin ?arm ?block ?p ?g ?q ?t)
            (Plate-supported ?block ?p ?plate)
            (AtAConf ?arm ?q)
            (not (UnsafePose ?block ?p))
            ;fluent-effect
            (AtGrasp ?arm ?block ?g)
        )
        :effect (and 
            (not (AtGrasp ?arm ?block ?g))
            (AtPose ?block ?p) 
            (HandEmpty ?arm)
            (On-plate ?block ?plate)
        )
    )
    (:action stack
        :parameters (?arm ?block ?lowerblock ?p ?g ?lowerp ?q ?t) ; ?rp 
        :precondition (and 
            ;type
            (Arm ?arm)
            (Block ?block)
            (Block ?lowerblock)
            (Pose ?block ?p)
            (Grasp ?block ?g)
            (Pose ?lowerblock ?lowerp)
            (AConf ?q)
            (ApproachMotion ?t)
            
            ;condition
            (Kin ?arm ?block ?p ?g ?q ?t)
            (Block-supported ?block ?p ?lowerblock ?lowerp)
            (not (= ?block ?lowerblock))
            (AtPose ?lowerblock ?lowerp)
            (AtAConf ?arm ?q)
            (not (UnsafePose ?block ?p))
            ;fluent-effect
            (AtGrasp ?arm ?block ?g)
            (Clear ?lowerblock)
        )
        :effect (and
            (not (AtGrasp ?arm ?block ?g))
            (AtPose ?block ?p) 
            (HandEmpty ?arm)
            (not (Clear ?lowerblock))
            (On-block ?block ?lowerblock)
        )
    )
    (:action unstack
        :parameters (?arm ?block ?lowerblock ?p ?g ?q ?t) ; ?rp 
        :precondition (and 
            ;type
            (Arm ?arm)
            (Block ?block)
            (Block ?lowerblock)
            (Pose ?block ?p)
            (Grasp ?block ?g)
            (AConf ?q)
            (ApproachMotion ?t)
            ;condition
            (Kin ?arm ?block ?p ?g ?q ?t)
            (not (= ?block ?lowerblock))
            (AtAConf ?arm ?q)
            (not (UnsafeApproach ?block ?p ?g))
            ;fluent-effect
            (Clear ?block)
            (AtPose ?block ?p)
            (HandEmpty ?arm)
            (On-block ?block ?lowerblock)
        )
        :effect (and
            (not (AtPose ?block ?p))
            (not (HandEmpty ?arm))
            (AtGrasp ?arm ?block ?g)
            (Clear ?lowerblock)
            (not (On-block ?block ?lowerblock))
        )
    )

)