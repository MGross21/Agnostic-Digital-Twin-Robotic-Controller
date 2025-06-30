# Glovebox Digital Twin

This project integrates the [mujoco-toolbox](https://github.com/MGross21/mujoco-toolbox) and [armctl](https://github.com/MGross21/armctl) repositories. Developed as part of a master's thesis at Arizona State University.

![Digital Twin Final Demonstration](assets/videos/dt.gif)

## Installation Instructions

> [!Note]
> For non-Windows users, installation steps may vary slightly. Adjust the commands according to your operating system.

Python Version: `3.11`

```bash
winget install --id=Python.Python.3.11 -e
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
flowchart TD
    Camera["Camera
• Image Capture"] --> Computer["Computer
• Object Pose Detection"]
    Computer --> Controller["Controller [armctl]
• Send Commands
• Robot Status"]
    Controller --> Computer
    Computer --> Simulation["Simulation [mujoco-toolbox]
• Robot Update
• Environment Update"]
    Simulation --> Computer

    style Camera stroke:#000000
    style Computer stroke:#000000
    style Controller stroke:#000000
    style Simulation stroke:#000000
```

## Visual Representation

![Glovebox UR5 Vention /w Cup](assets/images/ur5_vention_w_cup.png)

## Physical Implementation

| **View**          | **Before**                                      | **After**                                       |
|--------------------|------------------------------------------------|------------------------------------------------|
| **Regular Camera** | ![Init Camera](assets/images/final_testing/cup_init_camera.png) | ![Final Camera](assets/images/final_testing/cup_final_camera.png) |
| **Flat**           | ![Init Flat](assets/images/final_testing/cup_init_flat.png)     | ![Final Flat](assets/images/final_testing/cup_final_flat.png)     |
| **Sim**            | ![Init Sim](assets/images/final_testing/cup_init_sim.png)       | ![Final Sim](assets/images/final_testing/cup_final_sim.png)       |
| **External**       | ![Init External](assets/images/final_testing/external_reach_init.png)| ![Final External](assets/images/final_testing/external_reach_final.png)|

### Cup Location Tracking

![Cup Tracker](assets/figures/final_testing/ur5_cup_drag.gif)
