(define (domain kitchen)
  (:requirements :strips :typing)

  (:types
    location  ; locations in the kitchen
    item      ; any item in the kitchen
  )

  (:predicates
  )

  ; moves an item between two locations
  (:action grasp
    :parameters (?l - location ?i - item)
    :precondition (and (empty ?r) (gripperat ?l) (itemat(?i ?l))
    :effect (and (at ?r ?to) (not (occupied ?from)) (occupied ?to) (not (at ?r ?from)))
  )
  (:action move
    :parameters (?from - location ?to - location)
    :precondition (and (adjacent ?from ?to) (at ?r ?from) (not (occupied ?to)))
    :effect (and (at ?r ?to) (not (occupied ?from)) (occupied ?to) (not (at ?r ?from)))
  )
)
