(define (domain ObjectCollection-domain)
    
    (:predicates
        (can-move ?from-waypoint ?to-waypoint)
        (waypoint ?waypoint)
        (robot ?robot)
        (empty ?robot)
        (object ?object)
        (carry ?robot ?object)
        (at ?robot ?waypoint)
        (been-at ?robot ?waypoint)
        (is-in ?object ?waypoint)
        (bin-for ?waypoint ?object)
        (has-bin ?waypoint)

    )
    
    (:action move
        :parameters 
            (?robot
             ?from-waypoint 
             ?to-waypoint)

        :precondition 
            (and 
                (robot ?robot)
                (waypoint ?from-waypoint)
                (waypoint ?to-waypoint) 
                (at ?robot ?from-waypoint)
                (can-move ?from-waypoint ?to-waypoint))

        :effect 
            (and 
                (at ?robot ?to-waypoint)
                (been-at ?robot ?to-waypoint)
                (not (at ?robot ?from-waypoint)))
    )

    (:action pick-object
        :parameters 
            (?robot 
             ?object 
             ?waypoint)

        :precondition 
            (and 
                (robot ?robot)
                (object ?object)
                (waypoint ?waypoint) 
                (is-in ?object ?waypoint)
                (at ?robot ?waypoint)
                (empty ?robot))

        :effect 
            (and 
                (not (is-in ?object ?waypoint))
                (carry ?robot ?object)
                (not (empty ?robot)))
    )

    (:action place-object
        :parameters 
            (?robot
             ?object
             ?waypoint
            )
        :precondition 
            (and 
                (robot ?robot)
                (object ?object)
                (waypoint ?waypoint)
                (carry ?robot ?object)
                (has-bin ?waypoint)
                (bin-for ?waypoint ?object)
                (at ?robot ?waypoint)
        )
        :effect 
            (and 
                (not (carry ?robot ?object))
                (is-in ?object ?waypoint)
                (empty ?robot))
        )
    
)