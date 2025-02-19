import asyncio
import socket
import json
import logging
from abc import ABC, abstractmethod
import math
import numpy as np


logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class RobotController(ABC):
    async def __aenter__(self):
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_value, traceback):
        await self.disconnect()


    def __format__(self, format_spec):
        if format_spec == "f":
            return f"{self.__class__.__name__}({self.ip}:{self.port})"

    def __init__(self, ip, port):
        self.ip, self.port, self.socket = ip, port, None
        self.isConnected = False

    async def connect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            await asyncio.get_event_loop().sock_connect(self.socket, (self.ip, self.port))
            logger.info(f"Connected to {self:f}")
        except Exception as e:
            logger.error(f"Connection failed: {e}")
            raise ConnectionError(f"Failed to connect to {self:f}")
        self.isConnected = True

    async def disconnect(self):
        if self.socket:
            self.socket.close()
            self.socket = None
            logger.info("Disconnected from robot")
        self.isConnected = False

    async def send_command(self, command, timeout=5.0):
        if not self.socket:
            raise ConnectionError(f"Not connected to {self:f}")
        
        try:
            if isinstance(command, dict):
                command = json.dumps(command).encode('utf-8')
                response_format = 'dict'
            elif isinstance(command, str):
                command = command.encode('utf-8')
                response_format = 'str'
            elif isinstance(command, bytes):
                response_format = 'bytes'
            else:
                raise TypeError("Unsupported command type")
            
            logger.info(f"Sending command to {self:f}: \t{command}")
            await asyncio.wait_for(self._send(command), timeout=timeout)
            response = await asyncio.wait_for(self._receive(), timeout=timeout)
            
            if response_format == 'dict':
                decoded_response = json.loads(response.decode('utf-8'))
            elif response_format == 'str':
                decoded_response = response.decode('utf-8')
            elif response_format == 'bytes':
                decoded_response = response
            else:
                raise ValueError("Unsupported response format")
            
            logger.info(f"Received response from {self:f}: \t{decoded_response}")
            return decoded_response
    
        except asyncio.TimeoutError:
            logger.error("Command timed out")
            raise TimeoutError("Command timed out")
        except Exception as e:
            logger.error(f"Error sending command: {e}")
            raise ConnectionError("Failed to send command to the robot")

    async def _send(self, data):
        await asyncio.get_event_loop().sock_sendall(self.socket, data)

    async def _receive(self, buffer_size=1024):
        return await asyncio.get_event_loop().sock_recv(self.socket, buffer_size)
    
    @abstractmethod
    async def sleep(self, seconds): pass

    @abstractmethod
    async def move_joints(self, joint_positions,*args, **kwargs): pass

    @abstractmethod
    async def get_joint_positions(self): pass

    @abstractmethod
    async def move_cartesian(self, robot_pose, *args, **kwargs): pass

    @abstractmethod
    async def get_cartesian_position(self): pass

    @abstractmethod
    async def stop_motion(self): pass

    @abstractmethod
    async def get_robot_state(self): pass

    async def custom_command(self, command):
        return await self.send_command(command)

class ElephantRobotics(RobotController):
    def __init__(self, ip:str, port:int):
        super().__init__(ip, port)
        self.isConnected = False

    async def connect(self):
        await super().connect() # Socket Connection

        assert await self.send_command("power_on()") == "power_on:[ok]" # Power on the robot
        assert await self.send_command("state_on()") == "state_on:[ok]" # enable the system
        self.isConnected = True
    
    async def disconnect(self):
        await self.stop_motion() # Stop any ongoing motion
        # assert await self.send_command("state_off()") == "state_off:[ok]" # Shut down the system, but the robot is still powered on
        # assert await self.send_command("power_off()") == "power_off:[ok]" # Power off the robot
        await super().disconnect() # Socket disconnection
        self.isConnected = False
    
    async def sleep(self, seconds):
        await self.send_command(f"wait({seconds})")
        asyncio.sleep(seconds)
    
    async def move_joints(self, joint_positions, *args, **kwargs):
        """
        Move the robot to the specified joint positions.
        
        Parameters
        ----------
        joint_positions : list of float
            Joint positions in degrees [j1, j2, j3, j4, j5, j6].
        speed : int, optional
            Speed of the movement, range 0 ~ 2000 (default: 200).
        DOF : int, optional
            Degrees of freedom (default: 6).
        """
        
        if len(joint_positions) != kwargs.get("DOF", 6):
            raise ValueError("Joint positions must have 6 elements")
        
        joint_ranges = [
            (-180.00, 180.00),
            (-270.00, 90.00),
            (-150.00, 150.00),
            (-260.00, 80.00),
            (-168.00, 168.00),
            (-174.00, 174.00)
        ]
        for i, (low, high) in enumerate(joint_ranges):
            if not (low <= joint_positions[i] <= high):
                raise ValueError(f"Joint {i+1} angle out of range: {low} ~ {high}")

        speed = kwargs.get("speed", 200)
        if not (0 <= speed <= 2000):
            raise ValueError("Speed out of range: 0 ~ 2000")
        
        command = "set_angles"
        response =  await self.send_command(f"{command}({','.join(map(str, joint_positions))},{speed})")

        if response != f"{command}:[ok]":
            logger.error(f"Failed to move joints: {response}")
            raise ValueError("Invalid joint positions or speed")

    async def move_cartesian(self, robot_pose, *args, **kwargs):
        speed = kwargs.get("speed", 200)
        if not (0 <= speed <= 2000):
            raise ValueError("Speed out of range: 0 ~ 2000")
        if len(robot_pose) != 6:
            raise ValueError("Robot pose must have 6 elements: [x, y, z, rx, ry, rz]")
        
        command = f"set_coords({','.join(map(str, robot_pose))},{speed})"
        
        assert await self.send_command(command) == "set_coords:[ok]"
        
        while True:
            if await self.send_command("wait_command_done()", timeout=60) == "wait_command_done:0":
                break
            await asyncio.sleep(0.25)

    async def get_joint_positions(self):
        response = await self.send_command("get_angles()")
        if response == "[-1.0, -2.0, -3.0, -4.0, -1.0, -1.0]":
            raise ValueError("Invalid joint positions response from robot")
        return response

    async def get_cartesian_position(self):
        response = await self.send_command("get_coords()") # [x, y, z, rx, ry, rz]
        if response == "[-1.0, -2.0, -3.0, -4.0, -1.0, -1.0]":
            raise ValueError("Invalid cartesian position response from robot")
        cartesian_position = list(map(float, response[response.index("[")+1:response.index("]")].split(","))) # From string list to float list
        return np.array(cartesian_position)

    async def stop_motion(self):
        command = "task_stop"
        response = await self.send_command(f"{command}()")

        if not response.startswith(f"{command}:"):
            raise SystemError(f"Unexpected response: {response}")

        result = response.split(":", 1)[1]

        if result != "[ok]":
            raise SystemError(result)
        return True

    async def get_robot_state(self):
        command = "check_running"
        response = await self.send_command(f"{command}()")

        if not response.startswith(f"{command}:"):
            raise SystemError(f"Unexpected response format: {response}")

        status = response.partition(":")[2]  # Get everything after the colon

        if status == "1":
            return True
        elif status == "0":
            return False
        else:
            raise ValueError(f"Unknown robot state: {status}")

class UniversalRobotics(RobotController):
    def __init__(self, ip:str, port:int):
        super().__init__(ip, port)

    async def sleep(self, seconds):
        await self.send_command(f"sleep({seconds})")

    async def move_joints(self, joint_positions, *args, **kwargs)->str:
        """
        MoveJ
        --------
        Move the robot to the specified joint positions.

        Parameters
        ----------
        joint_positions : list of float
            Joint positions in radians [j1, j2, j3, j4, j5, j6].
        speed : float, optional
            Speed of the movement in rad/s (default: 0.1).
        acceleration : float, optional
            Acceleration of the movement in rad/s^2 (default: 0.0).
        time : float, optional
            The time in seconds to make the move. If specified, the command will ignore the speed and acceleration values (default: 0.0).
        r : float, optional
            Blend radius in meters (default: 0.0).
        DOF : int, optional
            Degrees of freedom (default: 6).
        """
        if len(joint_positions) != kwargs.get("DOF", 6):
            raise ValueError("Joint positions must have 6 elements")

        v = kwargs.get("speed", 0.1)
        assert v < 2, "Speed out of range: 0 ~ 2" # Source: https://forum.universal-robots.com/t/maximum-axis-speed-acceleration/13338/4
        
        a = kwargs.get("aceleration", 0.0)
        assert a <= 10, "Acceleration out of range: 0 ~ 10" # Source: https://forum.universal-robots.com/t/maximum-axis-speed-acceleration/13338/2
        # OR
        t = kwargs.get("time", 0.0)

        r = kwargs.get("r", 0.0)

        for pos in joint_positions:
            if not (0 <= pos <= math.pi*2):
                raise ValueError(f"Joint position {pos} out of range: 0 ~ {math.pi*2}")
            
        command = f"movej(p[{','.join(map(str, joint_positions))}], a={a}, v={v}, t={t}, r={r})\n"
        return await self.send_command(command)

    async def move_cartesian(self, robot_pose, *args, **kwargs)->str:
        """
        Move the robot to the specified cartesian position.
        Robot Pose: [x, y, z, rx, ry, rz] in meters and radians
        moveType: movel (linear cartesian pathing) or movep (circular cartesian pathing) (default: movel)
        v: velocity (Rad/s)
        a: acceleration (rad/s^2)
        t: The time (seconds) to make move is not specified. If it were specified the command would ignore the a and v values.
        r: Blend radius (m)
        """
        moveType = kwargs.get("moveType", "movel")
        assert moveType in ["movel", "movep"], "Unsupported move type: movel or movep"

        v = kwargs.get("speed", 0.1)
        assert v < 2, "Speed out of range: 0 ~ 2" # Source: https://forum.universal-robots.com/t/maximum-axis-speed-acceleration/13338/4
        
        a = kwargs.get("aceleration", 0.0)
        assert a <= 10, "Acceleration out of range: 0 ~ 10" # Source: https://forum.universal-robots.com/t/maximum-axis-speed-acceleration/13338/2
        # OR
        t = kwargs.get("time", 0.0)

        r = kwargs.get("r", 0.0)

        for pos in robot_pose[3:]:
            if not (0 <= pos <= math.pi*2):
                raise ValueError(f"Joint position {pos} out of range: 0 ~ {math.pi*2}")
            
        if self.send_command("is_within_safety_limits({})".format(','.join(map(str, robot_pose))) == "False"):
            raise ValueError("Cartesian position out of safety limits")

        command = f"{moveType}(p[{','.join(map(str, robot_pose))}], a={a}, v={v}, t={t}, r={r})\n"
        return await self.send_command(command)

    async def get_joint_positions(self): pass

    async def get_cartesian_position(self): pass

    async def stop_motion(self):
        return await self.send_command("stopj(2)") # deceleration: 2 rad/s^2

    async def get_robot_state(self): pass

# Non-Operational (1/31/2025)
class Fanuc(RobotController):
    def __init__(self, ip:str, port:int):
        super().__init__(ip, port)

    async def move_joints(self, joint_positions, speed=1.0):
        return await self.send_command({"type": "move_joints", "positions": joint_positions, "speed": speed})

    async def move_cartesian(self, x, y, z, rx, ry, rz, speed=1.0):
        return await self.send_command({"type": "move_cartesian", "position": [x, y, z, rx, ry, rz], "speed": speed})

    async def get_joint_positions(self):
        return await self.send_command({"type": "get_joint_positions"})

    async def get_cartesian_position(self):
        return await self.send_command({"type": "get_cartesian_position"})

    async def stop_motion(self):
        return await self.send_command({"type": "stop"})

    async def get_robot_state(self):
        return await self.send_command({"type": "get_state"})

class AgnosticController:
    controllers = {}
    for manufacturer in RobotController.__subclasses__():
        controllers[manufacturer.__name__.lower()] = manufacturer

    def __init__(self, manufacturer, ip, port):
        if isinstance(manufacturer, str):
            manufacturer = manufacturer.strip().lower()
            if manufacturer not in self.controllers:
                raise ValueError(f"Unsupported manufacturer: {manufacturer}")
            self.controller = self.controllers[manufacturer](ip, port)
        
        elif issubclass(manufacturer, RobotController):
            self.controller = manufacturer(ip, port)

        else:
            raise ValueError(f"Invalid manufacturer type. Supported types: str, RobotController subclasses ({', '.join(self.controllers.keys())})")

    async def __aenter__(self):
        await self.controller.connect()
        return self.controller

    async def __aexit__(self, exc_type, exc_value, traceback):
        await self.controller.disconnect()


async def main():
    async with AgnosticController(ElephantRobotics,"192.168.1.159", 5001) as pro600:
        # await pro600.move_cartesian([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        # await pro600.sleep(2)
        # await pro600.move_joints([1, 0, 0, 0, 0, 0], 0.5)
        # cart_pos = np.array(await pro600.get_cartesian_position())
        # move_up = np.array([25,25,25,0,np.pi/4,0])
        await pro600.move_cartesian([-276,165,350,-9,-19,-2], speed=400)
        await pro600.sleep(2)
        await pro600.get_cartesian_position()


if __name__ == "__main__":
    asyncio.run(main())