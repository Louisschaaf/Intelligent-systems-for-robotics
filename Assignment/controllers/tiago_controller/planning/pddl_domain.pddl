(define (domain hall_nav)
  (:requirements :typing :strips)
  (:types robot place)
  (:predicates
    (at ?r - robot ?p - place)
    (next ?p1 - place ?p2 - place)
  )

  (:action goto-next
    :parameters (?r - robot ?from - place ?to - place)
    :precondition (and (at ?r ?from) (next ?from ?to))
    :effect (and (at ?r ?to) (not (at ?r ?from)))
  )
)