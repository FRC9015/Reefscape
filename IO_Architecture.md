# IO Architecture Overview

The IO Architecture for the Reefscape project is designed to handle various subsystems and their interactions with the robot hardware and simulation environments. This architecture is crucial for the robot's performance in the FIRST Robotics Competition.

## Notable Features (Working)

### Autonomous Improvements
- **2 Piece L4 Auto**: Autonomous mode capable of handling two game pieces.
- **Auto align to Source**: Automatic alignment to game piece sources.

### Control & Mechanics
- **IO Interface for Subsystems**: Integration with both hardware and simulation environments.
- **PID, Feedforward, Voltage-Based Motor Control**: Advanced motor control techniques for precise movements.
- **Tuned Drivetrain**: Optimized for performance and reliability.
- **Elevator + Intake Combo**: Efficient system for handling game pieces, featuring sensors for coral detection.
- **Tuned End Effector**: Optimized for game piece manipulation.
- **Manual Override for Automated Presets**: Allows manual control during automated sequences.
- **Button Board with Scoring Presets**: Simplifies scoring operations with predefined settings.

### Vision & Localization
- **Odometry and Updates based on AprilTag Detection**: Accurate robot positioning using visual markers.
- **2-Camera Pose Estimation**: Enhanced localization and pose estimation using dual cameras.

### Data & Debugging
- **Full Logging and Simulation of Data**: Comprehensive data logging and simulation for debugging and analysis.

## Features in Development

- **Climber Mechanism**: Under development for endgame climbing tasks.
- **Algae/De-Algae Mechanisms**: Mechanisms for handling specific game elements.
- **3, 4 Piece L4 Auto**: Advanced autonomous routines for handling more game pieces.
- **Full Reef Autoalign**: Complete automatic alignment system.
- **Elevator Limit Switch + Sensor**: Additional safety and control features for the elevator system.
- **3D Animation and Physics Simulation**: Enhanced simulation capabilities.
- **Coral Detection Using Machine Learning**: Advanced object detection using AI.
- **3 Camera Pose Estimation**: Further improvements to localization using an additional camera.

## Recent Changes

- **Auton Code**: Enhanced autonomous code by [@REVSIX](https://github.com/REVSIX) in [#22](https://github.com/FRC9015/Reefscape/pull/22).
- **Elevator Updates**: Improvements by [@levinator123](https://github.com/levinator123) in [#24](https://github.com/FRC9015/Reefscape/pull/24) and [#25](https://github.com/FRC9015/Reefscape/pull/25).
- **Pre-Week 1 Auto & Path Changes**: Updates by [@levinator123](https://github.com/levinator123) in [#26](https://github.com/FRC9015/Reefscape/pull/26).
- **Mount Olive Competition Code**: Competition-specific code by [@Omkaarcodes](https://github.com/Omkaarcodes) in [#27](https://github.com/FRC9015/Reefscape/pull/27).

For more details, refer to the [full changelog](https://github.com/FRC9015/Reefscape/compare/v0.1.0-weekZero...v1.0.0-week2).

---

*This document provides a brief overview of the IO Architecture and the current state of features based on the latest release notes.*