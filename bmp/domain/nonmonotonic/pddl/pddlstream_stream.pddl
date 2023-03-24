(define (stream pr2-tamp)
  (:stream sample-pose
    :inputs (?o ?r)
    :domain (Stackable ?o ?r)
    :outputs (?p)
    :certified (and (Pose ?o ?p) (Supported ?o ?p ?r))
  )
  (:stream sample-grasp
    :inputs (?o)
    :domain (Graspable ?o)
    :outputs (?g)
    :certified (Grasp ?o ?g)
  )

  (:stream inverse-kinematics
    :inputs (?a ?o ?p ?g)
    :domain (and (Arm ?a) (Pose ?o ?p) (Grasp ?o ?g)) ;(Controllable ?a) 
    :outputs (?q ?t)
    :certified (and (AConf ?q) (ATraj ?t) (Kin ?a ?o ?p ?g ?q ?t)) ; 
  )
  (:stream plan-arm-motion
    :inputs (?a ?q1 ?q2)
    :domain (and (Arm ?a) (AConf ?q1) (AConf ?q2))
    ;:fluents (AtPose AtGrasp) ; AtBConf
    :outputs (?t)
    :certified (and (ATraj ?t) (ArmMotion ?a ?q1 ?t ?q2))
  )
  ; (:stream plan-approach-motion
  ;     :inputs (?a ?o ?p ?g ?q)
  ;     :domain (Kin a? ?o ?p ?g ?q)
  ;     ;:fluents (AtPose AtGrasp)
  ;     :outputs (?t)
  ;     :certified (ApproachMotion ?q ?t ?o ?p ?g)
  ; )
  ; (:stream plan-base-motion
  ;   :inputs (?q1 ?q2)
  ;   :domain (and (BConf ?q1) (BConf ?q2))
  ;   ;:fluents (AtPose AtGrasp) ; AtBConf
  ;   :outputs (?t)
  ;   :certified (and (BTraj ?t) (BaseMotion ?q1 ?t ?q2))
  ; )

  (:stream test-cfree-pose-pose
    :inputs (?o1 ?p1 ?o2 ?p2)
    :domain (and (Pose ?o1 ?p1) (Pose ?o2 ?p2))
    :certified (CFreePosePose ?o1 ?p1 ?o2 ?p2)
  )
  (:stream test-cfree-approach-pose
    :inputs (?o1 ?p1 ?g1 ?o2 ?p2)
    :domain (and (Pose ?o1 ?p1) (Grasp ?o1 ?g1) (Pose ?o2 ?p2))
    :certified (CFreeApproachPose ?o1 ?p1 ?g1 ?o2 ?p2)
  )
  (:stream test-cfree-traj-pose
    :inputs (?t ?o2 ?p2)
    :domain (and (ATraj ?t) (Pose ?o2 ?p2))
    :certified (CFreeTrajPose ?t ?o2 ?p2)
  )
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