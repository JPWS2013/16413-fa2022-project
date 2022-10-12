(define (domain boxmoving)
  (:requirements :strips)

  (:types
    truck     
    box
    location
  )

  (:predicates
  	(boxon ?t - truck ?b - box)
  	(boxin ?b - box ?l - location)
  	(truckin ?t - truck ?l - location)
  )

  
  (:action drive
    :parameters (?src - location ?dest - location ?t - truck)
    :precondition (and(truckin ?t ?src))
    :effect (and (truckin ?t ?dest) (not (truckin ?t ?src)))
  )
  
  (:action load
    :parameters (?b - box ?t - truck ?l - location)
    :precondition (and (boxin ?b ?l) (truckin ?t ?l))
    :effect (and (boxon ?b ?t) (not (boxin ?b ?l)))
  )
  
  (:action unload
    :parameters (?b - box ?t - truck ?l - location)
    :precondition (and (boxon ?b ?t) (truckin ?t ?l))
    :effect (and (boxin ?b ?l) (not (boxon ?b ?t)))
  )
)
