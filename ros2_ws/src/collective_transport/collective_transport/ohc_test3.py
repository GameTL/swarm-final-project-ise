import os
import threading
import time
import signal
import sys

from submodules.p2p_communication.communicator import Communicator

# Setup
communicator = Communicator(identifier="jetson3")

# Flag for clean shutdown
running = True

def signal_handler(sig, frame):
    global running
    print("\n[INFO] Shutting down gracefully...")
    running = False

# Attach signal handler (Ctrl+C will trigger this)
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# Start the communicator server thread
server_thread = threading.Thread(target=communicator.comm_thread_spawner, daemon=True)
server_thread.start()

# Main loop: keep alive until interrupted
try:
    while running:
        time.sleep(0.5)  # Small sleep to avoid busy waiting
except Exception as e:
    print(f"[ERROR] Exception in main thread: {e}")
finally:
    print("[INFO] Main program exiting. (Daemon thread will be killed automatically.)")
    sys.exit(0)
