import os
import threading
import time
import signal
import sys

from submodules.p2p_communication.communicator import Communicator

ROBOT_ID: int = int(os.environ.get('ROBOT_ID'))
communicator = Communicator(identifier="jetson1")

running = True

def signal_handler(sig, frame):
    global running
    print("\n[INFO] Signal received. Shutting down...")
    running = False
    communicator.stop()

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# Start the server
server_thread = threading.Thread(target=communicator.comm_thread_spawner, daemon=True)
server_thread.start()

# Main control loop
try:
    while running:
        user_input = input("Press 'S' to claim taskmaster role or 'Q' to quit: ").strip().upper()
        if user_input == "S":
            communicator.object_detected()
            communicator.cleanup()
        elif user_input == "Q":
            running = False
            communicator.stop()
        else:
            print("[WARN] Unknown command.")
except Exception as e:
    print(f"[ERROR] Exception: {e}")
finally:
    print("[INFO] Exiting main program.")
    sys.exit(0)
