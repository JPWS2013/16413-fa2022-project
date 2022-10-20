(define (domain kitchen)
  (:requirements :strips :typing)
  
  (:types
    location   ; Locations in the kitchen
    item       ; Objects in the kitchen
    arm        ; the robotic arm 
  )
  
  (:predicates
    (opened ?l - location)
    (closed ?l - location)
    (openable ?l - location)
    (gripped ?i - item)
    (free ?i - item)
    (itemat ?i - item ?l - location)
    (armat ?a - arm ?l - location)
    (empty ?a - arm)
  )
  
  (:action open
    :parameters (?a - arm ?l - location)
    :precondition (and (armat ?a ?l) (openable ?l) (empty ?a))
    :effect (and (opened ?l) (not (closed ?l)))
  )
  
  (:action close
    :parameters (?a - arm ?l - location)
    :precondition (and (armat ?a ?l) (openable ?l) (empty ?a) (opened ?l))
    :effect (and (closed ?l) (not (opened ?l)))
  )
  
  (:action grip
    :parameters (?a - arm ?i - item ?l - location)
    :precondition (and (empty ?a) (free ?i) (armat ?a ?l) (itemat ?i ?l)) ; Might need to check if location is opened first before gripping (e.g. if we're taking something out of a drawer)
    :effect (and (gripped ?i) (not (itemat ?i ?l)) (not (free ?i)) (not (empty ?a)) )
  )
  
  (:action placein
    :parameters (?a - arm ?i - item ?l - location)
    :precondition (and (gripped ?i) (armat ?a ?l) (opened(?l)))
    :effect (and (free ?i) (itemat ?i ?l) (empty ?a) (not (gripped ?i)) )
  )

  (:action placeon
    :parameters (?a - arm ?i - item ?l - location)
    :precondition (and (gripped ?i) (armat ?a ?l))
    :effect (and (free ?i) (itemat ?i ?l) (empty ?a) (not (gripped ?i)) )
  )
  
  (:action move
    :parameters (?a - arm ?src - location ?dest - location)
    :precondition (and (armat ?a ?src))
    :effect (and (armat ?a ?dest) (not(armat ?a ?src)) )
  )
 )
