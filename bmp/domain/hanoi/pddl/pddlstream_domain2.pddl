(define (domain pr2-tamp)
  (:requirements :strips :equality :disjunctive-preconditions :negative-preconditions :derived-predicates)
  ;(:constants @sink @stove)
  (:predicates
    ;static predicates
    (Arm ?arm)
    (Disc ?disc)
    (Peg ?peg)
    
    ;
    (Pose ?disc ?p)
    (Grasp ?disc ?g)
    (ApproachMotion ?t)
    (AConf ?q)
    (ATraj ?t)
    (Smaller ?smaller ?larger)

    ;(Stackable ?o ?r)
    (Kin ?arm ?disc ?p ?g ?q ?t)
    (ArmMotion ?arm ?q1 ?t ?q2)
    (Peg-supported ?disc ?p ?peg)
    (Disc-supported ?disc ?p ?lowerdisc ?lowerp)

    ;fluents
    (HandEmpty ?arm)
    (Empty-peg ?peg) ;??
    (AtAConf ?arm ?q)
    (AtPose ?disc ?p)
    (AtGrasp ?arm ?disc ?g)
    (Clear ?disc)
    (On-peg ?disc ?peg)
    (On-disc ?disc ?lowerdisc)

    ;test
    (CFreeApproachPose ?disc ?p ?g ?disc2 ?p2)
    (CFreeTrajPose ?t ?disc2 ?p2)
    (UnsafeApproach ?disc ?p ?g)
    (UnsafeATraj ?t)
  )
    (:derived (UnsafeApproach ?disc ?p ?g)
        (exists (?disc2 ?p2) 
            (and 
                (Pose ?disc ?p) 
                (Grasp ?disc ?g) 
                (Pose ?disc2 ?p2) 
                (not (= ?disc ?disc2))
                (not (CFreeApproachPose ?disc ?p ?g ?disc2 ?p2))
                (AtPose ?disc2 ?p2)
            ))
    )
    (:derived (UnsafeATraj ?t)
        (exists (?disc2 ?p2) 
            (and 
                (ATraj ?t)
                (Pose ?disc2 ?p2)
                (not (CFreeTrajPose ?t ?disc2 ?p2))
                (AtPose ?disc2 ?p2)))
    )

    (:action pick
        :parameters (?arm ?disc ?peg ?p ?g ?q ?t) ; ?rp 
        :precondition (and 
            ;type
            (Arm ?arm)
            (Disc ?disc)
            (Peg ?peg)
            (Pose ?disc ?p)
            (Grasp ?disc ?g)
            (AConf ?q)
            (ApproachMotion ?t)
            
            ;fluent-static
            (Kin ?arm ?disc ?p ?g ?q ?t)
            (Clear ?disc)
            (AtAConf ?arm ?q)
            (not (UnsafeApproach ?disc ?p ?g))
            
            ;fluent-effect
            (AtPose ?disc ?p)
            (HandEmpty ?arm)
            (On-peg ?disc ?peg)
        )
        :effect (and 
            (not (AtPose ?disc ?p)) 
            (AtGrasp ?arm ?disc ?g)
            (not (HandEmpty ?arm))
            (not (On-peg ?disc ?peg))
            (Empty-peg ?peg)
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
    :parameters (?arm ?disc ?peg ?p ?g ?q ?t) ; ?rp 
    :precondition (and 
            ;type
            (Arm ?arm)
            (Disc ?disc)
            (Peg ?peg)
            (Pose ?disc ?p)
            (Grasp ?disc ?g)
            (AConf ?q)
            (ApproachMotion ?t)
            ;condition
            (Kin ?arm ?disc ?p ?g ?q ?t)
            (Peg-supported ?disc ?p ?peg)
            (AtAConf ?arm ?q)
            (not (UnsafeApproach ?disc ?p ?g))
            ;fluent-effect
            (AtGrasp ?arm ?disc ?g)
            (Empty-peg ?peg)   
        )
        :effect (and 
            (not (AtGrasp ?arm ?disc ?g))
            (AtPose ?disc ?p) 
            (HandEmpty ?arm)
            (not (Empty-peg ?peg))
            (On-peg ?disc ?peg)
        )
    )
    (:action stack
        :parameters (?arm ?disc ?lowerdisc ?p ?g ?lowerp ?q ?t) ; ?rp 
        :precondition (and 
            ;type
            (Arm ?arm)
            (Disc ?disc)
            (Disc ?lowerdisc)
            (Pose ?disc ?p)
            (Grasp ?disc ?g)
            (Pose ?lowerdisc ?lowerp)
            (AConf ?q)
            (ApproachMotion ?t)
            
            ;condition
            (Kin ?arm ?disc ?p ?g ?q ?t)
            (Disc-supported ?disc ?p ?lowerdisc ?lowerp)
            (Smaller ?disc ?lowerdisc)
            (not (= ?disc ?lowerdisc))
            (AtPose ?lowerdisc ?lowerp)
            (AtAConf ?arm ?q)
            (not (UnsafeApproach ?disc ?p ?g))
            ;fluent-effect
            (AtGrasp ?arm ?disc ?g)
            (Clear ?lowerdisc)
        )
        :effect (and
            (not (AtGrasp ?arm ?disc ?g))
            (AtPose ?disc ?p) 
            (HandEmpty ?arm)
            (not (Clear ?lowerdisc))
            (On-disc ?disc ?lowerdisc)
        )
    )
    (:action unstack
        :parameters (?arm ?disc ?lowerdisc ?p ?g ?q ?t) ; ?rp 
        :precondition (and 
            ;type
            (Arm ?arm)
            (Disc ?disc)
            (Disc ?lowerdisc)
            (Pose ?disc ?p)
            (Grasp ?disc ?g)
            (AConf ?q)
            (ApproachMotion ?t)
            ;condition
            (Kin ?arm ?disc ?p ?g ?q ?t)
            (not (= ?disc ?lowerdisc))
            (Smaller ?disc ?lowerdisc)
            (AtAConf ?arm ?q)
            (not (UnsafeApproach ?disc ?p ?g))
            ;fluent-effect
            (Clear ?disc)
            (AtPose ?disc ?p)
            (HandEmpty ?arm)
            (On-disc ?disc ?lowerdisc)
        )
        :effect (and
            (not (AtPose ?disc ?p))
            (not (HandEmpty ?arm))
            (AtGrasp ?arm ?disc ?g)
            (Clear ?lowerdisc)
            (not (On-disc ?disc ?lowerdisc))
        )
    )

)