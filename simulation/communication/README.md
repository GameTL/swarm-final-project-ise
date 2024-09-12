# Communication in Swarm Intelligence

We should start simulating these 2 prerequisites on a few robots (e.g. 5) as a starting point

## Requirements

### 1. Message Transmission

Members of the swarm fleet should be aware of others' coordinates, done via networked communication

#### Constraints:
- Robots are stationary
- Robots are on fixed coordinates

#### Acceptance Criteria:
- Awareness of other members' coordinates

### 2. P2P Collision Avoidance

Evidence as to members' awareness of one another, random movement will be required but can be omitted as we can utilize existing frameworks 

#### Constraints:
- Random movement exists
- Empty room, no obstacles because no available sensors

#### Acceptance Criteria:
- Proximity repulsion (Priority)

<hr />

## Backlog / Stretch

- Consensus
- Architectural Design for communication designed for scalability -> Attach a diagram
- Modes for communication: -> State Diagram
  - Idle (Random movement with proximity repulsion)
  - Collaborative (No repulsion once an object is detected)
- Formation & Recovery for defects