(define (stream pr2-tamp)
  (:stream sample-plate-place-pose
    :inputs (?block ?plate) ;
    :domain (and (Block ?block) (Plate ?plate)) ;
    :outputs (?p)
    :certified (and (Pose ?block ?p) (Plate-supported ?block ?p ?plate)) ; 
  )
  (:stream sample-block-place-pose
    :inputs (?block ?lowerblock ?lowerp) ;
    :domain (and (Block ?block) (Block ?lowerblock) (Pose ?lowerblock ?lowerp)) ;
    :outputs (?p)
    :certified (and (Pose ?block ?p) (Block-supported ?block ?p ?lowerblock ?lowerp)) ; 
  )

  (:stream sample-grasp
    :inputs (?block)
    :domain (Block ?block)
    :outputs (?g)
    :certified (Grasp ?block ?g)
  )

  (:stream inverse-kinematics
    :inputs (?arm ?block ?p ?g)
    :domain (and (Arm ?arm) (Block ?block) (Pose ?block ?p) (Grasp ?block ?g))
    :outputs (?q ?t)
    :certified (and (AConf ?q) (ApproachMotion ?t) (Kin ?arm ?block ?p ?g ?q ?t)) ; 
  )

  (:stream plan-arm-motion
    :inputs (?arm ?q1 ?q2)
    :domain (and (Arm ?arm) (AConf ?q1) (AConf ?q2))
    ;:fluents (AtPose AtGrasp AtAConf) ; AtBConf
    :outputs (?t)
    :certified (and (ATraj ?t) (ArmMotion ?arm ?q1 ?t ?q2))
  )
  
  (:stream test-cfree-pose-pose
    :inputs (?block ?p ?block2 ?p2)
    :domain (and (Pose ?block ?p) (Pose ?block2 ?p2))
    :certified (CFreePosePose ?block ?p ?block2 ?p2)
  )
  (:stream test-cfree-approach-pose
    :inputs (?block ?p ?g ?block2 ?p2)
    :domain (and (Pose ?block ?p) (Grasp ?block ?g) (Pose ?block2 ?p2))
    :certified (CFreeApproachPose ?block ?p ?g ?block2 ?p2)
  )
  (:stream test-cfree-traj-pose
    :inputs (?t ?block2 ?p2)
    :domain (and (ATraj ?t) (Pose ?block2 ?p2))
    :certified (CFreeTrajPose ?t ?block2 ?p2)
  )
;   (:stream test-kin
;     :inputs (?o ?p ?r ?rp ?g ?q)
;     :domain (and (Supported ?o ?p ?r ?rp) (AConf ?q) (Grasp ?o ?g))
;     :certified (KinCheck ?o ?p ?r ?rp ?g ?q)
;   )
  ;(:stream test-cfree-traj-grasp-pose
  ;  :inputs (?t ?a ?o1 ?g1 ?o2 ?p2)
  ;  :domain (and (BTraj ?t) (Arm ?a) (Grasp ?o1 ?g1) (Pose ?o2 ?p2))
  ;  :certified (CFreeTrajGraspPose ?t ?a ?o1 ?g1 ?o2 ?p2)
  ;)

  ; (:function (Distance ?q1 ?q2)
  ;   (and (AConf ?q1) (AConf ?q2))
  ; )
  ;(:function (MoveCost ?t)
  ;  (and (BTraj ?t))
  ;)
)