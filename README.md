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
    Camera --> Perception
    Computer --> Perception
    Perception --> Controller
    Controller --> Simulation

    DigitalTwin <|-- Perception
    DigitalTwin <|-- Controller
    DigitalTwin <|-- Simulation

    DigitalTwin : +start()
    DigitalTwin : +run()
    DigitalTwin : +shutdown()

    class Camera {
        +captureImage()
        +streamData()
    }

    class Computer {
        +processImages()
        +extractObjectPoses()
    }

    class Perception {
        +detectObjects()
        +estimatePose()
    }

    class Controller {
        +sendCommands()
        +readStatus()
        +inverseKinematics()
    }

    class Simulation {
        +simulateRobot()
        +simulateEnvironment()
    }
```
