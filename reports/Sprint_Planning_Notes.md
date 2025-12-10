# Sprint Planning Notes

**Project**: Swarm Construction with Multiple Agents
**Date**: December 2025

## Phase 1 Planning
**Goal**: Establish the foundation for the swarm robotics project, including simulation environment, CI/CD pipeline, and basic software architecture.

### Planned Tasks (1.x Series)
*   **Infrastructure & CI**:
    *   [Task 1.1] Set up GitHub CI with CodeCov integration and README badges.
    *   [Task 1.2] Clean up unused packages (`my_controller`, `my_model`) to maintain a clean workspace.
*   **Simulation Setup**:
    *   [Task 1.3] Implement Swarm simulation with 12 robots in a Webots environment using `config.yaml`.
    *   [Task 1.4] Create integration tests to verify camera and LiDAR topic publishing for all robots.
*   **Software Architecture**:
    *   [Task 1.5] Implement stub for Finite State Machine (FSM) in `swarm_model` package.
    *   [Task 1.6] Implement stub for Swarm Controller in `swarm_controller` package.
*   **Documentation & Quality**:
    *   [Task 1.7] Generate initial UML diagrams (Class and Sequence) and Doxygen documentation.
    *   [Task 1.8] Update test scripts (`do-tests-and-coverage.bash`) to include Phase 1 checks.
    *   [Task 1.9] Finalize Phase 1 deliverables and link them in the documentation.

## Phase 2 Planning
**Goal**: Implement autonomous behavior, object interaction, and robust testing to achieve full swarm functionality.

### Planned Tasks (2.x Series)
*   **Simulation Refinements**:
    *   [Task 2.1] Update launch files to parameterize the number of robots and rooms; adjust environment colors.
    *   [Task 2.2] Enhance object detection simulation (waste generation parameters, seeding).
    *   [Task 2.3] Add configuration for robot camera resolution.
    *   [Task 2.4] Set up autonomous initialization (spawn direction, static TF).
    *   [Task 2.15] Adjust object positioning for safety margins and graspability.
*   **Core Logic & FSM**:
    *   [Task 2.7] Complete FSM implementation with states: `FIND_OBJECT`, `APPROACH_OBJECT`, `GRASP_OBJECT`, `MOVE_HOME`, `MOVE_OUT`, `RELEASE_OBJECT`.
    *   [Task 2.8] Add unit tests for FSM transitions.
*   **Controller Implementation**:
    *   [Task 2.5] Launch controller nodes efficiently (MultiThreadedExecutor).
    *   [Task 2.6] Implement camera image subscription and processing (3x3 pixel array).
    *   [Task 2.9] Implement LiDAR subscription for obstacle avoidance.
    *   [Task 2.11] Implement relative position calculation and state logging.
    *   [Task 2.12] Implement `move_to_location` logic (rotate & move straight).
    *   [Task 2.17] Tune rotation speeds.
    *   [Task 2.18] Refactor `swarm_controller` for better testability (separate library).
*   **Object Interaction (Hardware/Simulation)**:
    *   [Task 2.13] Implement "Hoop" mechanism (linear actuator) for object manipulation.
    *   [Task 2.14] Integrate Hoop control into `robot_controller`.
*   **Verification & Documentation**:
    *   [Task 2.10] Add Supervisor node for ground truth verification.
    *   [Task 2.16] Update UML diagrams to reflect final architecture.
    *   [Task 2.19] Add comprehensive Doxygen comments to all files.
    *   [Task 2.20] Generate final Doxygen documentation.
    *   [Task 2.21] Update project README with final instructions and details.
