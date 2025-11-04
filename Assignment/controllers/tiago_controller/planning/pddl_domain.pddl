(define (domain home_assistant)
  (:requirements :typing :strips)
  (:types robot place item support)

  (:predicates
    (at ?x ?p)           ; x∈{robot,item} at place
    (on ?i ?s)           ; item on support
    (freehand ?r)        ; robot hand free
    (holding ?r ?i)
    (adjacent ?p1 ?p2)   ; navigatie topologie
  )

  (:action move
    :parameters (?r - robot ?from - place ?to - place)
    :precondition (and (at ?r ?from) (adjacent ?from ?to))
    :effect (and (at ?r ?to) (not (at ?r ?from))))

  (:action pickup
    :parameters (?r - robot ?i - item ?p - place ?s - support)
    :precondition (and (at ?r ?p) (at ?i ?p) (on ?i ?s) (freehand ?r))
    :effect (and (holding ?r ?i) (not (on ?i ?s))))

  (:action place_on
    :parameters (?r - robot ?i - item ?p - place ?s - support)
    :precondition (and (at ?r ?p) (holding ?r ?i))
    :effect (and (on ?i ?s) (freehand ?r) (not (holding ?r ?i))))
)