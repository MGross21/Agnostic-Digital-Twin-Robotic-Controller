import socket
import threading
import json
import struct
import sys
from typing import Callable, Optional, Any

class LocalPubSub:
    def __init__(self, port: Optional[int] = 5000, multicast_group: str = '239.0.0.1', multicast_port: int = 5000, max_history: int = 100):
        self.port = port
        self.multicast_group = multicast_group
        self.multicast_port = multicast_port
        self.max_history = max_history
        self._topics = {}
        self._callbacks = {}
        self._socket = None
        self._listener_thread = None
        self._running = False
        self._lock = threading.RLock()
    
    def _create_socket(self):
        """Create a UDP socket for multicast with timeout."""
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Works cross-platform
        self._socket.bind(('0.0.0.0', self.multicast_port))
        self._socket.settimeout(1.0)  # Prevent blocking forever

        # Join multicast group
        group = socket.inet_aton(self.multicast_group)
        mreq = struct.pack('4s4s', group, socket.inet_aton('0.0.0.0'))
        self._socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    
    def _run_listener(self):
        """Continuously listen for incoming multicast messages."""
        while self._running:
            try:
                data, addr = self._socket.recvfrom(4096)
                self._handle_message(data, addr)
            except socket.timeout:
                continue  # Avoid hanging
    
    def _handle_message(self, data: bytes, addr: tuple):
        """Handle incoming message."""
        try:
            message = json.loads(data.decode())
            topic = message.get('topic')
            payload = message.get('payload')

            with self._lock:
                if topic in self._topics:
                    self._topics[topic].append(payload)
                    if len(self._topics[topic]) > self.max_history:
                        self._topics[topic] = self._topics[topic][-self.max_history:]
                
                for callback in self._callbacks.get(topic, []):
                    try:
                        callback(payload)
                    except Exception as e:
                        print(f"Error in callback for topic {topic}: {e}")
        except json.JSONDecodeError:
            print(f"Malformed message received: {data}")
    
    def start_listener(self):
        """Start the multicast listener thread."""
        if self._running:
            return
        self._running = True
        self._create_socket()
        self._listener_thread = threading.Thread(target=self._run_listener, daemon=True)
        self._listener_thread.start()
    
    def stop_listener(self):
        """Stop the multicast listener thread and close the socket."""
        self._running = False
        if self._listener_thread:
            self._listener_thread.join(timeout=2)
        if self._socket:
            self._socket.close()
            self._socket = None
        print("LocalPubSub listener stopped.")

    def subscribe(self, topic: str, callback: Optional[Callable] = None) -> None:
        """Subscribe to a topic with an optional callback."""
        with self._lock:
            if topic not in self._topics:
                self._topics[topic] = []
            if callback:
                if topic not in self._callbacks:
                    self._callbacks[topic] = []
                self._callbacks[topic].append(callback)
    
    def unsubscribe(self, topic: str, callback: Optional[Callable] = None) -> None:
        """Unsubscribe from a topic."""
        with self._lock:
            if topic in self._callbacks:
                if callback:
                    self._callbacks[topic] = [cb for cb in self._callbacks[topic] if cb != callback]
                else:
                    self._callbacks[topic] = []

    def publish(self, topic: str, message: Any) -> None:
        """Publish a message to a topic."""
        try:
            if self._socket:
                payload = json.dumps({'topic': topic, 'payload': message}).encode()
                self._socket.sendto(payload, (self.multicast_group, self.multicast_port))
        except Exception as e:
            print(f"Error publishing message: {e}")

    def shutdown(self):
        """Shutdown and exit the program cleanly."""
        print("Shutting down LocalPubSub...")
        self.stop_listener()
        sys.exit(0)

    def __enter__(self):
        self.start_listener()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.shutdown()

    def __eq__(self, other):
        if not isinstance(other, LocalPubSub):
            return False
        return (self.port == other.port and
                self.multicast_group == other.multicast_group and
                self.multicast_port == other.multicast_port)

    def __lshift__(self, data_tuple):
        """Use the << operator to publish a message."""
        topic, message = data_tuple
        self.publish(topic, message)
        return self

    def __rshift__(self, callback):
        """Use the >> operator to subscribe to a topic."""
        topic, callback_function = callback
        self.subscribe(topic, callback_function)
        return self

    def __repr__(self):
        return f"LocalPubSub(port={self.port}, multicast_group={self.multicast_group}, multicast_port={self.multicast_port})"

    def __str__(self):
        return f"LocalPubSub: Listening on {self.multicast_group}:{self.multicast_port} with port {self.port}"


if __name__ == "__main__":
    try:
        import time
        pubsub = LocalPubSub()
        pubsub.start_listener()

        def callback(msg):
            print(f"Received: {msg}")

        pubsub.subscribe("test_topic", callback)

        # Simulating message publishing
        pubsub.publish("test_topic", "Hello, Multicast!")

        # Keep the script running
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        pubsub.shutdown()
