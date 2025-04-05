# Agnostic Digital Twin Robotic Controller

## Installation Instructions

Python Version: `3.11.3`

```bash
git clone https://github.com/MGross21/Agnostic-Digital-Twin-Robotic-Controller
cd Agnostic-Digital-Twin-Robotic-Controller
chmod +x setup_and_run.sh
./setup_and_run.sh
```

## Core Structure of Digital Twin

```mermaid
classDiagram
    DigitalTwin <|-- Controller
    DigitalTwin <|-- Simulation

    DigitalTwin : +start()
    DigitalTwin : +run()
    DigitalTwin : +shutdown()

    class Controller {
        +sendCommands()
        +readStatus()
    }

    class Simulation {
        +sendCommands()
        +readStatus()
    }

    Camera --> Simulation
    Camera --> Controller

    class Camera {
        +detectObjects()
    }
```