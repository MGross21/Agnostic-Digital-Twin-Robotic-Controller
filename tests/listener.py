import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from digital_twin import LocalPubSub


listener = LocalPubSub(port=5000)

listener.subscribe("test", lambda msg: print(f"Received message: {msg}"))