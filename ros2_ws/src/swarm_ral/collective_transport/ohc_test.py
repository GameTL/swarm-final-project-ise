import os
import threading 

from submodules.p2p_communication.communicator import Communicator

ROBOT_ID: int = int(os.environ.get('ROBOT_ID'))
communicator = Communicator(identifier=ROBOT_ID)

server_thread = threading.Thread(target=communicator.comm_thread_spawner, daemon=True)
server_thread.start()

# communicator.current_coords = data # (x, y)