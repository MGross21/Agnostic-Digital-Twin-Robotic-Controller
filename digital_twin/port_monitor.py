import os
import sys
import socket
import struct
import psutil
import time
import platform
import json

class PortMonitor:
    def __init__(self, multicast_ip: str = "224.0.0.1", multicast_port: int = 5000, interval: float = 2.0, whitelist: dict = None):
        """
        Monitors open public ports (1024-49151) and listens for multicast messages.

        :param multicast_ip: Multicast IP address to listen on.
        :param multicast_port: Multicast port to listen on.
        :param interval: Time interval (seconds) between updates for port monitoring.
        :param whitelist: Dictionary of criteria to filter by {'ports':[], 'pids':[], 'process':[]} (optional).
        """
        self.multicast_ip = multicast_ip
        self.multicast_port = multicast_port
        self.interval = interval
        self.whitelist = whitelist or {"ports": [], "pids": [], "process": []}

        # Initialize the socket for multicast listening
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind(('0.0.0.0', self.multicast_port))

        # Join the multicast group
        group = socket.inet_aton(self.multicast_ip)
        mreq = struct.pack('4sL', group, socket.INADDR_ANY)
        self._socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    def clear_screen(self):
        """Clears the terminal screen for a refreshing effect."""
        os.system("cls" if platform.system() == "Windows" else "clear")

    def is_whitelisted(self, port_info):
        """
        Checks if the port, pid, or process name matches the whitelist criteria.
        
        :param port_info: Dictionary with port, pid, and process information.
        :return: True if the port_info matches any whitelist criteria, otherwise False.
        """
        # Check if port is in whitelist
        if port_info["port"] in self.whitelist["ports"]:
            return True
        
        # Check if PID is in whitelist
        if port_info["pid"] in self.whitelist["pids"]:
            return True
        
        # Check if process name is in whitelist
        if port_info["process"] in self.whitelist["process"]:
            return True
        
        return False

    def get_public_ports(self):
        """
        Retrieves a list of open ports in the public range (1024-49151)
        along with their associated process IDs, filtered by whitelist criteria.
        """
        public_ports = []
        
        for conn in psutil.net_connections(kind="inet"):
            if conn.laddr and conn.status == psutil.CONN_LISTEN and (1024 <= conn.laddr.port <= 49151):
                process = psutil.Process(conn.pid) if conn.pid else None
                port_info = {
                    "port": conn.laddr.port,
                    "pid": conn.pid,
                    "process": process.name() if process else "Unknown"
                }

                # Check if the port info matches any whitelist criteria
                if self.is_whitelisted(port_info):
                    public_ports.append(port_info)

        # Sort ports by port number
        public_ports.sort(key=lambda x: x["port"])
        return public_ports

    def listen_to_multicast(self):
        """
        Continuously listens to multicast messages on the multicast IP and port.
        """
        try:
            while True:
                data, addr = self._socket.recvfrom(4096)
                if data:
                    try:
                        message = json.loads(data.decode())
                        print(f"Received multicast message: {message}")
                    except json.JSONDecodeError:
                        print(f"Received malformed multicast message: {data}")
        except KeyboardInterrupt:
            print("\nMulticast listening stopped.")
            self._socket.close()

    def display_live_ports(self):
        """
        Continuously displays public ports (filtered by whitelist) and updates in real-time with a clear screen.
        """
        try:
            while True:
                self.clear_screen()  # Clear screen before displaying new data
                public_ports = self.get_public_ports()

                print("\n--- Live Public Port Usage (1024-49151) ---")
                print("{:<10} {:<10} {}".format("Port", "PID", "Process"))
                print("-" * 40)

                if public_ports:
                    for port_info in public_ports:
                        print("{:<10} {:<10} {}".format(port_info["port"], port_info["pid"], port_info["process"]))
                else:
                    print("No public ports currently in use.")

                time.sleep(self.interval)
        except KeyboardInterrupt:
            self.clear_screen()
            print("\nMonitoring stopped.")

    def close(self):
        """Closes the socket when done."""
        self._socket.close()


# Example usage (if __name__ == "__main__")
if __name__ == "__main__":
    # Define a whitelist dictionary (for example, we allow port 8080, PID 1234, and process "python")
    whitelist = {
        "ports": [8080, 5000],
        "pids": [1234],
        "process": ["python"]
    }
    
    # Create a PortMonitor instance
    monitor = PortMonitor(interval=2, whitelist=whitelist)

    # Start the multicast listener in a separate thread
    import threading
    listener_thread = threading.Thread(target=monitor.listen_to_multicast, daemon=True)
    listener_thread.start()

    # Start monitoring live public ports
    monitor.display_live_ports()
