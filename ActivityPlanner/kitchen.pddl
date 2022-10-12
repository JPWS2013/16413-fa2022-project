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
    :parameters (?a - arm ?l - location ?a - robot)
    :precondition (and (armat ?a - arm ?l - location) (openable ?l - location) (empty ?a - arm))
    :effect (and (opened ?l - location) (not (closed ?l - location)))
  )
  
  (:action close
    :parameters (?a - arm ?l - location ?a - robot)
    :precondition (and (armat ?a - arm ?l - location) (openable ?l - location) (empty ?a - arm))
    :effect (and (closed ?l - location) (not (opened ?l - location)))
  )
  
  (:action grip
    :parameters (?a - arm ?i - item ?l - location)
    :precondition (and (empty ?a - arm) (free ?i - item) (armat ?a - arm ?l - location) (itemat ?i - item ?l - location)) ; Might need to check if location is opened first before gripping (e.g. if we're taking something out of a drawer)
    :effect (and (gripped ?i - item) (not (itemat ?i - item ?l - location)) (not (free ?i - item)) (not (empty ?a - arm)) )
  )
  
  (:action release
    :parameters (?a - arm ?i - item ?l - location)
    :precondition (and (gripped ?i - item) (armat ?a - arm ?l - location))
    :effect (and (free ?i - item) (itemat ?i - item ?l location) (empty ?a - arm) (not (gripped ?i - item)) )
  )
  
  (:action move
    :parameters (?a - arm ?src - location ?dest - location)
    :precondition (and (armat ?a - arm ?src - location))
    :effect (and (armat ?a - arm ?dest - location) (not(armat ?a - arm ?src - location)) )
  )
 )
