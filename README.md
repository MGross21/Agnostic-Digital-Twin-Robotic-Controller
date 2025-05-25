# Glovebox Digital Twin

This project integrates the [Mujoco Toolbox](https://github.com/MGross21/mujoco-toolbox) and [armctl](https://github.com/MGross21/armctl) repositories. Developed as part of a master's thesis at Arizona State University.

## Installation Instructions

> [!Note]
> For non-Windows users, installation steps may vary slightly. Adjust the commands according to your operating system.

Python Version: `3.11.3`

```bash
git clone https://github.com/MGross21/glovebox-digitaltwin
cd glovebox-digitaltwin
py -3.11 -m venv .venv
.\.venv\Scripts\activate
python.exe -m pip install --upgrade pip
pip install poetry
poetry install
```

## Core Structure of Digital Twin

```mermaid
classDiagram
    %% UDP Topics:
    %% Camera --> Computer: /camera/object_pose
    %% Computer --> Controller: /controller/command (pub)
    %% Controller --> Computer: /controller/status (pub)
    %% Computer --> Simulation: /sim/update (pub)
    %% Simulation --> Computer: /sim/state (pub)

    Camera --> Computer
    Computer --> Controller
    Controller --> Computer
    Computer --> Simulation
    Simulation --> Computer

    class Camera {
        +captureImage()
        +detectObjectPose()
    }

    class Computer {
        +processCameraData()
    }

    class Controller {
        +sendCommands()
        +receiveStatus()
    }

    class Simulation {
        +updateRobot()
        +updateEnvironment()
    }
```
