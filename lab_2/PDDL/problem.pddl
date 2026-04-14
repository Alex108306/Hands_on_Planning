(define (problem ObjectCollection-1)

    (:domain 
        ObjectCollection-domain    
    )
    
    (:objects
        start wp1 wp2 wp3 wp4 wp5 soda_bin coke_bin - waypoint
        
        soda1 soda2 coke1 coke2
        
        robot1
    )
    
    (:init
        
        (waypoint start) (waypoint wp1) (waypoint wp2) (waypoint wp3)
        (waypoint wp4) (waypoint wp5) (waypoint soda_bin) (waypoint coke_bin)
        
        (object soda1) (object soda2) (object coke1) (object coke2)
        
        (is-in soda1 wp1) (is-in soda2 wp3) (is-in coke1 wp4) (is-in coke2 wp5)

        (has-bin soda_bin) (has-bin coke_bin)
        (bin-for coke_bin coke1) (bin-for coke_bin coke2)
        (bin-for soda_bin soda1) (bin-for soda_bin soda2)
        
        (can-move start wp1) (can-move wp1 wp2) (can-move wp2 wp3) (can-move wp3 wp4) 
        (can-move wp4 wp5) (can-move wp1 coke_bin) (can-move wp1 soda_bin)
        (can-move wp2 soda_bin) (can-move wp2 coke_bin) (can-move wp3 soda_bin)
        (can-move wp3 coke_bin) (can-move wp4 soda_bin) (can-move wp4 coke_bin)
        (can-move wp5 soda_bin) (can-move wp5 coke_bin) (can-move soda_bin wp1)
        (can-move coke_bin wp1) (can-move soda_bin wp2) (can-move coke_bin wp2) 
        (can-move soda_bin wp3) (can-move coke_bin wp3) (can-move soda_bin wp4)
        (can-move coke_bin wp4) (can-move soda_bin wp5) (can-move coke_bin wp5)

        (robot robot1)
        (empty robot1)
        (at robot1 start)
    )
    
    (:goal
        (and 
            (is-in soda1 soda_bin)
            (is-in soda2 soda_bin)
            (is-in coke1 coke_bin)
            (is-in coke2 coke_bin)
        )
    )
)