```mermaid
flowchart TD
    M[Mapping Node\n(grid_mapping)]
    O[Localization Node]
    F[Frontier Exploration Node\n(input: /map + /pose\noutput: /goal)]
    G[Global Planner Node\n(RRT*)]

    M -->|/map| F
    O -->|/pose| F
    F -->|/goal| G
```
