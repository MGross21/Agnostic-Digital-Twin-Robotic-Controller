# Glovebox Digital Twin

This project integrates the [mujoco-toolbox](https://github.com/MGross21/mujoco-toolbox) and [armctl](https://github.com/MGross21/armctl) repositories. Developed as part of a master's thesis at Arizona State University.

## Installation Instructions

> [!Note]
> For non-Windows users, installation steps may vary slightly. Adjust the commands according to your operating system.

Python Version: `3.11`

```bash
winget install -e --id Python.Python.3.11
```

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
graph TD
    Camera["Camera
+captureImage()
+detectObjectPose()"]

    Computer["Computer
+processCameraData()"]

    Controller["Controller [armctl]
+sendCommands()
+receiveStatus()"]

    Simulation["Simulation [mujoco-toolbox]
+updateRobot()
+updateEnvironment()"]

    Camera -->|/camera/object_pose| Computer
    Computer -->|/controller/command| Controller
    Controller -->|/controller/status| Computer
    Computer -->|/sim/update| Simulation
    Simulation -->|/sim/state| Computer
```
