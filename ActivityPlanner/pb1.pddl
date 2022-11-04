(define (problem pb1)
  (:domain kitchen)
  ;Note: hitman_countertop is the left countertop and indigo_countertop is the right countertop in the kitchen
  (:objects
    a - arm
    hitman_countertop back_left_stove back_right_stove front_left_stove front_right_stove indigo_countertop indigo_drawer_top start_pos - location
    sugar_box0 potted_meat_can1 - item
  )
  
  (:init
    (itemat sugar_box0 back_right_stove)
    (free sugar_box0)
    (itemat potted_meat_can1 indigo_countertop)
    (free potted_meat_can1)
    (closed indigo_drawer_top)
    (empty a)
    (armat a start_pos)
    (openable indigo_drawer_top) 
    (surface hitman_countertop)
    ;(surface indigo_countertop)
    ;(surface back_left_stove)
    ;(surface back_right_stove)
    ;(surface front_left_stove)
    ;(surface front_right_stove)
  )
  
  (:goal (and (itemin potted_meat_can1 indigo_drawer_top) (itemat sugar_box0 hitman_countertop) (closed indigo_drawer_top))) ;(free sugar_box0) (free potted_meat_can1)))
)
