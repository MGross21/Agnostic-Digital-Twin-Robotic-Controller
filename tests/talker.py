import os
import sys
import time

# Add the parent directory to the Python path
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from digital_twin import LocalPubSub

with LocalPubSub(port=5000) as talker:
    for i in range(5):
        # Publish a message to the "test" topic
        message = f"Hello, World! {i}"
        print(f"Publishing: {message}")
        talker.publish("test", message)
        time.sleep(1)