(define (problem logistics)
  (:domain boxmoving)
  
  (:objects
    t - truck
    c1 c2 c3 - location
    b - box
  )
  
  (:init
    (boxin b c1)
    (truckin t c2)
  )


  (:goal (and (boxin b c3)) )
)
