(define (problem pb1)
  (:domain kitchen)
  (:objects
    a - arm
    left_counter ul_burner ur_burner ll_burner lr_burner right_counter indigo_drawer_top start_pos - location
    sugar_box0 potted_meat_can1 - item
  )
  
  (:init
    (itemat sugar_box0 ur_burner)
    (free sugar_box0)
    (itemat potted_meat_can1 right_counter)
    (free potted_meat_can1)
    (closed indigo_drawer_top)
    (empty a)
    (armat a start_pos)
    (openable indigo_drawer_top) 
    (surface left_counter)
    ;(surface right_counter)
    ;(surface ul_burner)
    ;(surface ur_burner)
    ;(surface ll_burner)
    ;(surface lr_burner)
  )
  
  (:goal (and (itemin potted_meat_can1 indigo_drawer_top) (itemat sugar_box0 left_counter) (closed indigo_drawer_top))) ;(free sugar_box0) (free potted_meat_can1)))
)
