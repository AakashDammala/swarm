# Sprint Review Notes

**Project**: Swarm Construction with Multiple Agents
**Date**: December 2025

## Executive Summary
The sprint successfully concluded with the completion of both Phase 1 and Phase 2 objectives. The team moved from a basic simulation setup to a fully autonomous swarm capable of searching, grasping, and retrieving objects ("waste") in a simulated environment.

## Phase 1 Review
**Status**: Completed

### Key Achievements
*   **CI/CD Pipeline**: Successfully integrated CodeCov and GitHub Actions, ensuring verified builds.
*   **Simulation Baseline**: Established a stable Webots simulation supporting 12 robots, controlled via `config.yaml`.
*   **Architecture**: Defined the separation of concerns between `swarm_model` (FSM logic) and `swarm_controller` (ROS 2 nodes).
*   **Testing**: Verified sensor data availability (LiDAR, Camera) through integration tests.

## Phase 2 Review
**Status**: Completed

### Key Achievements
*   **Autonomous Behavior**:
    *   Implemented a robust Finite State Machine (FSM) handling the full lifecycle: searching, approaching, grasping, moving home, and releasing objects.
    *   Successfully implemented `move_to_location` logic allowing precise navigation.
*   **Object Manipulation**:
    *   Devised and implemented a "Hoop" mechanism acting as a linear actuator to physically grasp and transport objects in the simulation.
    *   Fine-tuned object placement and safety margins to ensure reliable grasping.
*   **Perception**:
    *   Integrated camera processing (3x3 center pixel analysis) for waste detection.
    *   Utilized LiDAR data for basic obstacle avoidance and approach logic.
*   **Refactoring & Quality**:
    *   Refactored `swarm_controller` into a shared library to facilitate unit testing.
    *   Added `ground_truth` supervisor node for verification.
    *   Completed comprehensive unit tests for geometric calculations and state conversions.
*   **Documentation**:
    *   Delivered complete UML diagrams (Class and Sequence).
    *   Generated full Doxygen documentation for the codebase.
    *   Finalized README with usage instructions.

### Challenges & Solutions
*   **Object Grasping**: Initial grasp logic was unreliable.
    *   *Solution*: Implemented a "Hoop" actuator mechanism and adjusted object spacing margins [Task 2.13, 2.15].
*   **Performance**: Simulating multiple robots with cameras was computationally expensive.
    *   *Solution*: Made camera resolution configurable and optimized processing [Task 2.3, 2.6].
*   **Testing**: Testing ROS nodes inside the binary was difficult.
    *   *Solution*: Refactored controller logic into a separate library to expose functions for GTest [Task 2.18].
