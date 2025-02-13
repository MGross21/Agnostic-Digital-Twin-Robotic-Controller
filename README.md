# Agnostic Digital Twin Robotic Controller

## Installation Instructions

```bash
git clone https://github.com/MGross21/Agnostic-Digital-Twin-Robotic-Controller
chmod +x setup_and_run.sh
./setup_and_run.sh
```

## First Test

```bash
2025-02-12 13:18:11,350 - INFO - Connected to 192.168.1.159:5001
2025-02-12 13:18:11,351 - INFO - Sending command: b'power_on()'
2025-02-12 13:18:11,954 - INFO - Received response: power_on:[ok]
2025-02-12 13:18:11,954 - INFO - Sending command: b'state_on()'
2025-02-12 13:18:12,647 - INFO - Received response: state_on:[ok]
2025-02-12 13:18:12,647 - INFO - Sending command: b'get_angles()'
2025-02-12 13:18:12,663 - INFO - Received response: get_angles:[0.290562,-95.891321,-74.804509,-162.949219,1.845703,12.041016]
2025-02-12 13:18:12,663 - INFO - Sending command: b'task_stop()'
2025-02-12 13:18:13,466 - INFO - Received response: task_stop:[ok]
2025-02-12 13:18:13,466 - INFO - Sending command: b'state_off()'
2025-02-12 13:18:14,176 - INFO - Received response: state_off:[ok]
2025-02-12 13:18:14,176 - INFO - Sending command: b'power_off()'
2025-02-12 13:18:14,176 - INFO - Received response: power_off:[ok]
2025-02-12 13:18:14,176 - INFO - Disconnected from robot
```