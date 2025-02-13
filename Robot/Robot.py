from dataclasses import dataclass, field
from typing import List, Union, Tuple
import numpy as np
from Utils import Transform, Math

@dataclass
class DH:
    """
    A class to represent Denavit-Hartenberg parameters for a robot link.
    
    Parameters
    ----------
    Specify tuple for a ranged parameter, e.g. (-pi, pi)
    OR
    Specify a single float value for a fixed parameter, e.g. 0.5
    """
    a: Union[float, Tuple[float, float]] # float or tuple
    alpha: Union[float, Tuple[float, float]] # float or tuple
    d: Union[float, Tuple[float, float]] # float or tuple
    theta: Union[float, Tuple[float, float]] # float or tuple
    
    type: str = field(init=False) # revolute, prismatic, fixed
    range: Union[Tuple[float, float], None] = field(init=False, default=None) 
    frame: np.ndarray = field(init=False) # transformation matrices for each joint
    reach: np.ndarray = field(init=False) # reachable workspace for robot arm

    def __getattr__(self, item):
        param = self.__dict__.get(item)
        if isinstance(param, tuple):
            return param[1] # return max of range by default
        return param

    def __post_init__(self):
        parameters = [self.theta, self.d, self.a, self.alpha]
        assert sum(len(param)>1 for param in parameters) <= 1, "Only one parameter may have a ranged tuple"

        if isinstance(self.d, tuple):
            self.type = 'revolute'
            self.range = self.theta
            self.reach = Transform.H(self.a, self.alpha, self.d, self.theta) # theta = (tuple,tuple)
            self.theta = 0

            
        elif isinstance(self.d, tuple):
            self.type = 'prismatic'
            self.range = self.d
            self.reach = Transform.H(self.a, self.alpha, self.d, self.theta) # d = (tuple,tuple)
            self.d = 0
        else:
            self.type = 'fixed'
            self.reach = Transform.H(self.a, self.alpha, self.d, self.theta) # none = 0

        self.frame = Transform.H(self.a, self.alpha, self.d, self.theta) # theta,d,none = 0

    def __repr__(self):
        return f"DH(a={self.a}, alpha={self.alpha}, d={self.d}, theta={self.theta}, type={self.type})"
    
    @staticmethod
    def from_json(json_file: str, key: str = 'DH_Parameters') -> Union[List['DH'], None]:
        try:
            with open(json_file, 'r') as file:
                import json
                data = json.load(file)
            
            chain = []
            for dh in data['robot'][key]:
                chain.append(DH(a=dh['a'], alpha=dh['alpha'], d=dh['d'], theta=dh['theta']))
            
            return chain
        except (IOError, json.JSONDecodeError) as e:
            print(f"Error reading or parsing the file: {e}")
            return None
        
    def xyz_pos(self):
        dh = self._dh_joint()
        return Transform.H(dh.a, dh.alpha, dh.d, dh.theta)[:3, 3]

    def _dh_joint(self):
        if len(self.theta) > 1:
            return DH(a=self.a, alpha=self.alpha, d=self.d, theta=0)
        elif len(self.d) > 1:
            return DH(a=self.a, alpha=self.alpha, d=0, theta=self.theta)
        else:
            return DH(a=self.a, alpha=self.alpha, d=self.d, theta=self.theta)


@dataclass
class Robot:
    """
    A class to represent a robot arm.
    
    Parameters
    ----------
    name : str
        The name of the robot arm.
    dh_params : list
        A list of DH parameters for each link in the robot arm.
    """
    manufacturer: str
    name: str
    dh_params: List[DH]
    ip: str
    port: int
    reach: np.ndarray = field(init=False)

    def __post_init__(self):
        self.reach = np.prod([dh.reach for dh in self.dh_params], axis=0)
        self.frame = np.eye(4)
        for dh in self.dh_params:
            self.frame = np.dot(self.frame, dh.frame)
        return self.reach
    
    def iterative_frames(chain: List['DH']) -> List[np.ndarray]:
        if not chain:
            return []
        frames = [chain[0].frame]
        for dh in chain[1:]:
            frames.append(frames[-1] @ dh.frame)




@dataclass
class UR5(Robot):
    dh_params = [
        DH(0, (-np.pi, np.pi), 0.089159, 0),
        DH(-0.425, 0, 0, (-np.pi/2, np.pi/2)),
        DH(-0.39225, 0, 0, 0),
        DH(0, 0.10915, 0, (-np.pi/2, np.pi/2)),
        DH(0, 0.09465, 0, (0, 2*np.pi)),
        DH(0, 0.0823, 0, (-np.pi/2, np.pi/2))
    ]
    ip = "192.168.1.100" # or 192.168.5.1
    port = 30002 
    dof = sum(1 for dh in dh_params if dh.type == 'revolute') + sum(1 for dh in dh_params if dh.type == 'prismatic')

    def __repr__(self):
        return f"{self.__name__}: {self.name}(dof={self.dof}, ip={self.ip}, port={self.port})"
        
@dataclass
class UniversalRobots:
    """
    Primary Interface: Port 30001, used for controlling the robot with high-level commands
    Secondary Interface: Port 30002, used for lower priority tasks and non-critical commands
    Realtime Interface: Port 30003, used for sending commands that require real-time execution and feedback from the robot
    
    """
    manufacturer: str = "Universal Robots"
    robots: dict = field(default_factory=dict)  

    def __post_init__(self):
        self.robots = {
            "UR5": UR5
        }

    def __contains__(self, item):
        return item in self.robots
    


if __name__ == "__main__":
    ur5 = UniversalRobots.robots['UR5'].dh_params
    print(ur5)