import socket
import threading
import json
from typing import Dict, List, Any, Callable, Optional, Set
import random

class LocalPubSub:
    _used_ports: Set[int] = set()
    _lock = threading.RLock()

    def __init__(self, port: Optional[int] = None, max_history: int = 100):
        self.port = port
        self.max_history = max_history
        self._topics: Dict[str, List[Any]] = {}
        self._callbacks: Dict[str, List[Callable]] = {}
        self._running = True
        self._closed = False
        self._lock = threading.RLock()
        self.instance_name = f"PubSub_{self.port}"

        with self._lock:
            self.port = self.port if self.port is not None else self._get_next_available_port()
            if self.port in self._used_ports:
                raise ValueError(f"Port {self.port} is already in use!")
            self._used_ports.add(self.port)
        
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind(('localhost', self.port))
        
        self._listener_thread = threading.Thread(target=self._run, daemon=True)
        self._listener_thread.start()

    def _get_next_available_port(self) -> int:
        default_ports = [5555, 5000, 6000, 8000]
        for port in default_ports:
            if port not in self._used_ports:
                return port
        
        while True:
            port = random.randint(1024, 65535)
            if port not in self._used_ports:
                return port

    def _run(self):
        while self._running:
            try:
                data, addr = self._socket.recvfrom(4096)
                self._handle_message(data, addr)
            except socket.timeout:
                continue
            except OSError:
                break

    def _handle_message(self, data: bytes, addr: tuple):
        try:
            message = json.loads(data.decode())
            topic = message.get('topic')
            payload = message.get('payload')
            
            if addr[1] == self.port:
                return
            
            with self._lock:
                if topic not in self._topics:
                    self._topics[topic] = []
                self._topics[topic].append(payload)
                
                if len(self._topics[topic]) > self.max_history:
                    self._topics[topic] = self._topics[topic][-self.max_history:]
                
                for callback in self._callbacks.get(topic, []):
                    try:
                        callback(payload)
                    except Exception as e:
                        print(f"Error in callback for topic {topic}: {e}")
        except json.JSONDecodeError:
            print(f"Received malformed message: {data}")
        except Exception as e:
            if self._running:
                print(f"Error handling message: {e}")

    @property
    def topics(self) -> Dict[str, List[Any]]:
        with self._lock:
            return {k: v[:] for k, v in self._topics.items()}

    @classmethod
    def ports(cls) -> Set[int]:
        with cls._lock:
            return cls._used_ports.copy()

    def publish(self, topic: str, message: Any) -> None:
        try:
            with self._lock:
                if topic not in self._topics:
                    self._topics[topic] = []
                self._topics[topic].append(message)
                
                if len(self._topics[topic]) > self.max_history:
                    self._topics[topic] = self._topics[topic][-self.max_history:]
                
                for callback in self._callbacks.get(topic, []):
                    try:
                        callback(message)
                    except Exception as e:
                        print(f"Error in callback for topic {topic}: {e}")
            
            payload = json.dumps({'topic': topic, 'payload': message}).encode()
            
            with self._lock:
                for port in self._used_ports:
                    if port != self.port:
                        self._socket.sendto(payload, ('localhost', port))
        except Exception as e:
            print(f"Error publishing message: {e}")

    def subscribe(self, topic: str, callback: Optional[Callable] = None) -> List[Any]:
        with self._lock:
            history = self._topics.get(topic, [])
            
            if callback:
                if topic not in self._callbacks:
                    self._callbacks[topic] = []
                self._callbacks[topic].append(callback)
                
            return history[:]

    def unsubscribe(self, topic: str, callback: Optional[Callable] = None) -> None:
        with self._lock:
            if topic in self._callbacks:
                if callback:
                    self._callbacks[topic] = [cb for cb in self._callbacks[topic] if cb != callback]
                else:
                    self._callbacks[topic] = []

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
        
        self._running = False
        
        if hasattr(self, '_listener_thread') and self._listener_thread.is_alive():
            self._listener_thread.join(timeout=1.0)
            
        if hasattr(self, '_socket') and self._socket:
            with self._lock:
                if self.port in self._used_ports:
                    self._used_ports.remove(self.port)
            try:
                self._socket.close()
                print(f"{self.instance_name} on port {self.port} shut down.")
            except Exception as e:
                print(f"Error closing socket: {e}")

    def __repr__(self) -> str:
        return f"{self.instance_name}(port={self.port})"

    def __del__(self):
        self.close()

    def __eq__(self, other):
        if not isinstance(other, LocalPubSub):
            return False
        return self.port == other.port

    def __rshift__(self, topic):
        return self.subscribe(topic)

    def __len__(self):
        with self._lock:
            return len(self._topics)

    def __contains__(self, topic):
        with self._lock:
            return topic in self._topics


if __name__ == "__main__":
    import time

    def print_message(msg):
        print(f"Received: {msg}")
    
    print("Starting pubsub1...")
    pubsub1 = LocalPubSub()
    pubsub1.subscribe("sensor_data", print_message)
    
    print("Starting pubsub2...")
    pubsub2 = LocalPubSub()
    
    pubsub1.publish("sensor_data", "temperature=25")
    pubsub2.publish("sensor_data", "humidity=60%")
    
    time.sleep(1)
    
    print(f"Used ports: {LocalPubSub.ports()}")
    
    # Test additional features
    print("Testing additional features...")
    
    # Test __len__
    print(f"Number of topics in pubsub1: {len(pubsub1)}")
    
    # Test __contains__
    print(f"Is 'sensor_data' in pubsub1? {'sensor_data' in pubsub1}")
    
    # Test unsubscribe
    pubsub1.unsubscribe("sensor_data", print_message)
    pubsub1.publish("sensor_data", "pressure=1013")
    
    time.sleep(1)
    
    pubsub1.close()
    pubsub2.close()