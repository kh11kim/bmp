(define (stream pr2-tamp)
  (:stream sample-peg-place-pose
    :inputs (?disc ?peg) ;
    :domain (and (Disc ?disc) (Peg ?peg)) ;
    :outputs (?p)
    :certified (and (Pose ?disc ?p) (Peg-supported ?disc ?p ?peg)) ; 
  )
  (:stream sample-disc-place-pose
    :inputs (?disc ?lowerdisc ?lowerp) ;
    :domain (and (Disc ?disc) (Disc ?lowerdisc) (Smaller ?disc ?lowerdisc) (Pose ?lowerdisc ?lowerp)) ;
    :outputs (?p)
    :certified (and (Pose ?disc ?p) (Disc-supported ?disc ?p ?lowerdisc ?lowerp)) ; 
  )

  (:stream sample-grasp
    :inputs (?disc)
    :domain (Disc ?disc)
    :outputs (?g)
    :certified (Grasp ?disc ?g)
  )

  (:stream inverse-kinematics
    :inputs (?arm ?disc ?p ?g)
    :domain (and (Arm ?arm) (Disc ?disc) (Pose ?disc ?p) (Grasp ?disc ?g))
    :outputs (?q ?t)
    :certified (and (AConf ?q) (ApproachMotion ?t) (Kin ?arm ?disc ?p ?g ?q ?t)) ; 
  )

  (:stream plan-arm-motion
    :inputs (?arm ?q1 ?q2)
    :domain (and (Arm ?arm) (AConf ?q1) (AConf ?q2))
    :fluents (AtPose AtGrasp AtAConf) ; AtBConf
    :outputs (?t)
    :certified (and (ATraj ?t) (ArmMotion ?arm ?q1 ?t ?q2))
  )
  
  ; (:stream plan-approach-motion
  ;     :inputs (?a ?o ?p ?g ?q)
  ;     :domain (Kin a? ?o ?p ?g ?q)
  ;     ;:fluents (AtPose AtGrasp)
  ;     :outputs (?t)
  ;     :certified (ApproachMotion ?a ?q ?t ?o ?p ?g)
  ; )
  ; (:stream plan-base-motion
  ;   :inputs (?q1 ?q2)
  ;   :domain (and (BConf ?q1) (BConf ?q2))
  ;   ;:fluents (AtPose AtGrasp) ; AtBConf
  ;   :outputs (?t)
  ;   :certified (and (BTraj ?t) (BaseMotion ?q1 ?t ?q2))
  ; )

;   (:stream test-cfree-pose-pose
;     :inputs (?o1 ?p1 ?o2 ?p2)
;     :domain (and (Pose ?o1 ?p1) (Pose ?o2 ?p2))
;     :certified (CFreePosePose ?o1 ?p1 ?o2 ?p2)
;   )
;   (:stream test-cfree-approach-pose
;     :inputs (?disc ?p ?g ?disc2 ?p2)
;     :domain (and (Pose ?disc ?p) (Grasp ?disc ?g) (Pose ?disc2 ?p2))
;     :certified (CFreeApproachPose ?disc ?p ?g ?disc2 ?p2)
;   )
;   (:stream test-cfree-traj-pose
;     :inputs (?t ?disc2 ?p2)
;     :domain (and (ATraj ?t) (Pose ?disc2 ?p2))
;     :certified (CFreeTrajPose ?t ?disc2 ?p2)
;   )
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