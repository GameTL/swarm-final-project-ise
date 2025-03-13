# Communicator Programmatic Flow

1. Robot starts, taking information from `hosts.json` and composing self priority queue <br>
2. Runs `comm_thread_spawner`, binding itself to the addr in the json and waits for connections using `handle_connection` <br>

### Object Detection and Consensus

3. Waits for robot to call the `object_detected` method, which `broadcast` a message of **OBJECT_DETECTED** header to the communication thread[2]
4. *TODO:* Robots should call a utility function to allow itself to stop moving
5. `handle_connection` calls `consensus` to assign the taskmaster role to itself
6. If robot is claiming `consensus` but there's another claiming, it'll access the priority queue give the higher one the role, then `broadcast` the taskmaster back to the swarm

### Path Planning : TODO

7. The swarm sends their own positions back to the taskmaster
8. Using the positions of each robot (received), the object and obstacle positions (from itself) it should call upon the path planning algorithm to compute collisionless paths for the swarm
9. Broadcast the paths back to the swarm and call a utility function to allow the robots to move according to the paths
- If approach is to simulate keyboard inputs, use motor speed to translate computed paths to sets of *keystrokes* and *duration they are pressed*

# Function Appendix

> `handle_connection` awaits messages in the format below, sending ***ACK*** and awaiting ***SYNACK*** to ensure reliability and handle retries
```
{
    header: the "topic" of the message
    sender: identifier of the sender (e.g. jetson1, jetson2, etc)
    content: content of the message such as positions
}
```
> `broadcast` is a utility function to send messages in the above format, having retries and ***ACK / SYNACK*** functionality

> `consensus` has a fallback functionality where if the final broadcast with the new taskmaster is conflicted, choose the one with the most votes -> constraint: swarm has to be an odd number
