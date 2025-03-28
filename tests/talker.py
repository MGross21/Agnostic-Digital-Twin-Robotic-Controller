import os
import sys

# Add the parent directory to the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
# Import the LocalPubSub class from the digital_twin module
from digital_twin import LocalPubSub

talker = LocalPubSub(port=5000)

talker.publish("test", "Hello, World!")