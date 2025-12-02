# Swarm: Autonomous Hazardous Waste Foraging System

![CICD Workflow status](https://github.com/AakashDammala/swarm/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg) [![codecov](https://codecov.io/gh/AakashDammala/swarm/branch/main/graph/badge.svg)](https://codecov.io/gh/AakashDammala/swarm) [![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

## Overview
Project Swarm is a decentralized multi-robot system designed to autonomously forage for hazardous waste (simulated as neon green objects) in a disaster scenario. The system utilizes multiple e-puck robots in a Webots simulation, leveraging ROS 2 Humble for communication and control.

## Team
**Group 5**
- Aakash Shetty Dammala
- Dayanidhi Kandade

## Features
- **Decentralized Swarm**: Scalable to 10+ robots.
- **Perception**: OpenCV-based color detection (HSV) and blob detection.
- **Behavior**: Finite State Machine (FSM) for searching, collecting, and dumping waste.
- **Simulation**: Webots simulation environment.

## Installation
1.  **Prerequisites**:
    - Ubuntu 22.04
    - ROS 2 Humble
    - Webots
    - Colcon

2.  **Build**:
    ```bash
    mkdir -p ~/swarm_ws/src
    cd ~/swarm_ws/src
    git clone https://github.com/AakashDammala/swarm.git .
    cd ..
    colcon build
    source install/setup.bash
    ```

## Usage
To launch the simulation with the swarm:
```bash
ros2 launch swarm_view swarm_launch.py
```

To run the integration tests for swarm_view package:
```bash
ros2 launch swarm_view test_robot_topics.launch.py
```

## Project Structure
```
src/
├── swarm_model/      # Model: Logic and State Management
│   ├── include/
│   └── src/
├── swarm_controller/ # Controller: Decision-making and Command Generation
│   ├── include/
│   └── src/
├── swarm_view/       # View: ROS 2 Interface and Sensors
│   ├── include/
│   └── src/
└── webots_ros2/      # Simulation packages
```

## MVC Architecture
The project follows the Model-View-Controller (MVC) architectural pattern to separate concerns and improve maintainability.

### Model (`swarm_model`)
The **Model** represents the internal state and logic of the robot. It is responsible for:
- Managing the Finite State Machine (FSM) (Idle, Searching, Collecting, Dumping, Avoiding).
- Processing sensor data to make decisions.
- Storing the state of the robot (e.g., current zone, object holding status).
- It does not interact directly with the hardware or simulation; it relies on the View for inputs and outputs.

### View (`swarm_view`)
The **View** acts as the interface between the Model and the external world (ROS 2 / Webots). It is responsible for:
- Reading sensor data (LiDAR, Camera, Odom) and passing it to the Controller.
- Receiving commands from the Controller and moving the robot accordingly.
- In this ROS 2 context, the View publishes the sensor topics and subscribes to command topics.

### Controller (`swarm_controller`)
The **Controller** uses the model to make decisions and orchestrates the flow of data.
- It ensures the Model updates its state based on new inputs from the View.
- It triggers the View to act based on the Model's decisions.

## Documentation

### Doxygen
To generate the API documentation:
```bash
doxygen Doxyfile
```
The documentation will be available in `docs/doxygen/html/index.html`.

### UML Diagrams
To generate UML diagrams using `clang-uml`:
```bash
clang-uml
```
The diagrams will be generated as `.puml` files in the `UML/` directory (e.g., `UML/swarm_class_diagram.puml`).

To convert the `.puml` files to PNG images (requires `plantuml`):
```bash
plantuml UML/*.puml
```
The generated PNG images will also be in the `UML/` directory.

## License
MIT