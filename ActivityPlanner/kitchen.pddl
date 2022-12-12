(define (domain kitchen)
  (:requirements :strips :typing)
  
  (:types
    location   ; Locations in the kitchen
    item       ; Objects in the kitchen
    arm        ; the robotic arm 
  )
  
  (:predicates
    (opened ?l - location)             ;Records if a location (e.g. a drawer) is opened
    (closed ?l - location)             ;Records if a location (e.g. a drawer) is closed
    (openable ?l - location)           ;Records if a location can be opened. This predicate differentiates countertops from drawers
    (gripped ?i - item)                ;Records if an item is being gripped by the robot arm
    (free ?i - item)                   ;Records if the item is not being gripped by the robot arm
    (itemat ?i - item ?l - location)   ;Records what location a specific item is at
    (itemin ?i - item ?l - location)   ;Records if an item is in an openable location
    (armat ?a - arm ?l - location)     ;Records what location the arm is at
    (empty ?a - arm)                   ;Records if the arm is holding an item
    (surface ?l - location)            ;Records if a location is a surface or not
  )
  
  ;This opens a location, provided that the arm is empty and at a location that is both openable and currently closed
  (:action open
    :parameters (?a - arm ?l - location)
    :precondition (and (armat ?a ?l) (openable ?l) (empty ?a) (closed ?l))
    :effect (and (opened ?l) (not (closed ?l)))
  )
  

  ;This closes a location, provided that the arm is empty and at a location that is openable and currently opened
  (:action close
    :parameters (?a - arm ?l - location)
    :precondition (and (armat ?a ?l) (openable ?l) (empty ?a) (opened ?l))
    :effect (and (closed ?l) (not (opened ?l)))
  )
  

  ;This causes an item to be gripped by the arm, provided that the arm is empty, the item is free and the arm and item are at the same location
  (:action grip
    :parameters (?a - arm ?i - item ?l - location)
    :precondition (and (empty ?a) (free ?i) (armat ?a ?l) (itemat ?i ?l) ) ; TODO: Might need to check if location is opened first before gripping (e.g. if we're taking something out of a drawer)
    :effect (and (gripped ?i) (not (itemat ?i ?l)) (not (free ?i)) (not (empty ?a)) )
  )
  

  ;This places the object in an openable location, provided that the item is gripped and the arm is at a location that is currently opened
  (:action placein
    :parameters (?a - arm ?i - item ?l - location)
    :precondition (and (gripped ?i) (armat ?a ?l) (opened ?l))
    :effect (and (free ?i) (itemat ?i ?l) (itemin ?i ?l) (empty ?a) (not (gripped ?i)) )
  )
  

  ;This places the object on a surface, provided that the item is gripped and the arm is at a location that is a surface
  (:action placeon
    :parameters (?a - arm ?i - item ?l - location)
    :precondition (and (gripped ?i) (armat ?a ?l) (surface ?l))
    :effect (and (free ?i) (itemat ?i ?l) (empty ?a) (not (gripped ?i)) )
  )
  

  ;This moves the arm from one location to another, provided that the arm is in the start location
  (:action move
    :parameters (?a - arm ?src - location ?dest - location)
    :precondition (and (armat ?a ?src))
    :effect (and (armat ?a ?dest) (not(armat ?a ?src)) )
  )
 )
