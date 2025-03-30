import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from digital_twin import LocalPubSub

# Subscribe to the "test" topic and define a callback to handle messages

with LocalPubSub(port=500) as listener:
    # Define a callback function to handle incoming messages
    def message_handler(msg):
        print(f"Received message: {msg}")

    # Subscribe to the "test" topic
    listener.subscribe("test", message_handler)

    # Keep the listener running
    try:
        while True:
            pass
    except KeyboardInterrupt:
        print("Listener stopped.")