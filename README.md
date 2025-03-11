# Reefscape

Welcome to the Reefscape repository! This project is developed by FRC Team 9015 for the FIRST Robotics Competition. Reefscape is the codebase for our robot, designed to compete in various challenges and tasks.

## Table of Contents

- [Introduction](#introduction)
- [Getting Started](#getting-started)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)
- [IO Architecture](#io-architecture)
- [Setting Up FRC Code on Your Laptop](#setting-up-frc-code-on-your-laptop)

## Introduction

Reefscape is the main software that powers our robot. It is designed to handle the complex tasks and challenges of the FIRST Robotics Competition. The code is written in Java and leverages the WPILib library to interface with the robot's hardware.

## Getting Started

To get started with Reefscape, you'll need to set up your development environment and clone this repository.

### Prerequisites

- Java Development Kit (JDK) 11 or higher
- Gradle
- WPILib

### Clone the Repository

```bash
git clone https://github.com/frc9015/reefscape.git
cd reefscape
```

## Installation

Follow these steps to set up the project on your local machine:

1. Ensure you have the required prerequisites installed.
2. Clone the repository using the command above.
3. Open the project in your preferred IDE (we recommend using Visual Studio Code or IntelliJ IDEA).
4. Build the project using Gradle:

```bash
./gradlew build
```

## Usage

To deploy the code to your robot, use the following command:

```bash
./gradlew deploy
```

Make sure your robot is connected and turned on before deploying.

## Contributing

We welcome contributions to the Reefscape project! If you have any ideas, bug fixes, or improvements, feel free to create a pull request. Please ensure that your contributions adhere to our coding standards and guidelines.

### Steps to Contribute

1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Commit your changes with descriptive commit messages.
4. Push your branch to your forked repository.
5. Create a pull request to merge your changes into the main repository.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.

## IO Architecture

### Overview

The IO Architecture for the Reefscape project is designed to handle various subsystems and their interactions with the robot hardware and simulation environments. This architecture is crucial for the robot's performance in the FIRST Robotics Competition.

### Notable Features (Working)

- **Autonomous Improvements**: 2 Piece L4 Auto, Auto align to Source
- **Control & Mechanics**: IO Interface for Subsystems with Hardware and Simulation Implementation, PID, Feedforward, Voltage-Based Motor Control, Tuned Drivetrain, Fast and Reliable Elevator + Intake Combo (With sensors for coral detection), Tuned End Effector, Manual Override for Automated Presets, Button Board with Scoring Presets
- **Vision & Localization**: Odometry and Updates based on AprilTag Detection, 2-Camera Pose Estimation
- **Data & Debugging**: Full Logging and Simulation of Data

### Features in Development

- Climber Mechanism
- Algae/De-Algae Mechanisms
- 3, 4 Piece L4 Auto
- Full Reef Autoalign
- Elevator Limit Switch + Sensor
- 3D Animation and Physics Simulation
- Coral Detection Using Machine Learning
- 3 Camera Pose Estimation

### Recent Changes

- **Auton Code**: Enhanced autonomous code by [@REVSIX](https://github.com/REVSIX) in [#22](https://github.com/FRC9015/Reefscape/pull/22).
- **Elevator Updates**: Improvements by [@levinator123](https://github.com/levinator123) in [#24](https://github.com/FRC9015/Reefscape/pull/24) and [#25](https://github.com/FRC9015/Reefscape/pull/25).
- **Pre-Week 1 Auto & Path Changes**: Updates by [@levinator123](https://github.com/levinator123) in [#26](https://github.com/FRC9015/Reefscape/pull/26).
- **Mount Olive Competition Code**: Competition-specific code by [@Omkaarcodes](https://github.com/Omkaarcodes) in [#27](https://github.com/FRC9015/Reefscape/pull/27).

For more details, refer to the [full changelog](https://github.com/FRC9015/Reefscape/compare/v0.1.0-weekZero...v1.0.0-week2).

## Setting Up FRC Code on Your Laptop

### A Comprehensive Guide

Programming a robot for the FIRST Robotics Competition (FRC) can seem daunting, but with the right tools and setup, it becomes a manageable and even enjoyable process. This guide will walk you through the essential steps to get your laptop ready for FRC programming, ensuring you have everything you need to start coding your robot.

### Step 1: Install the FRC Game Tools

The FRC Game Tools are a suite of software components necessary for programming and operating your robot. These include the FRC Driver Station, roboRIO Imaging Tool, and LabVIEW updates.

1. **Download the FRC Game Tools**: Navigate to the official FRC website and download the latest version of the FRC Game Tools. Ensure your laptop meets the system requirements, particularly running Windows 10 or higher with an x86 CPU[8].
2. **Uninstall Old Versions**: If you have previous versions of the FRC Game Tools installed, it’s recommended to uninstall them before proceeding. This ensures compatibility with the latest software[8].
3. **Install the Tools**: Run the installer and follow the on-screen instructions. Be prepared to update or install .NET Framework 4.6.2 if prompted[8].

### Step 2: Install WPILib and Visual Studio Code

WPILib is the library used for FRC programming, and Visual Studio Code (VS Code) is the recommended Integrated Development Environment (IDE).

1. **Download WPILib**: Visit the WPILib documentation site and download the WPILib installer for your operating system[9].
2. **Install WPILib**: Run the installer, which will set up WPILib and its dependencies, including VS Code[9].
3. **Configure VS Code**: Once installed, open VS Code and install the WPILib extension. This extension provides templates and tools specifically designed for FRC programming[1].

### Step 3: Install Vendor Libraries

If your robot uses components from specific vendors, such as Cross the Road Electronics (CTRE) or REV Robotics, you’ll need to install their respective libraries.

1. **CTRE Libraries**: Download the Phoenix Tuner and installer from the CTRE website. This tool allows you to configure and update CTRE devices like motors and sensors[1].
2. **REV Libraries**: For REV Robotics components, install the REV Hardware Client. This software helps manage and configure REV devices[5].

### Step 4: Set Up Your Development Environment

With all the necessary tools installed, it’s time to configure your development environment.

1. **Create a Workspace**: In VS Code, create a new workspace for your FRC projects. This will keep your code organized and accessible[4].
2. **Manage Vendor Libraries**: Use the WPILib extension to install and manage vendor libraries. This ensures your code can interact with the specific hardware components on your robot[1].
3. **Configure the roboRIO**: Use the roboRIO Imaging Tool to set up your robot’s brain. This tool allows you to update firmware and configure startup settings[8].

### Step 5: Test Your Setup

Before diving into coding, it’s crucial to ensure everything is working correctly.

1. **Connect to the Robot**: Use the FRC Driver Station to establish a connection with your robot. Verify that all components are recognized and functioning[3].
2. **Run a Test Program**: Deploy a simple program to your robot to confirm that your setup is correct. This could be a basic motor control or sensor readout[7].

### Additional Resources

For more detailed instructions and troubleshooting, refer to the following resources:

- [WPILib Documentation](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/index.html)
- [FRC Game Tools Installation Guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html)
- [CTRE Phoenix Tuner](https://www.ctr-electronics.com/talon-srx.html)
- [REV Hardware Client](https://docs.revrobotics.com/rev-hardware-client/)

By following these steps, you’ll have a fully functional FRC programming environment on your laptop, ready to tackle the challenges of the competition. Happy coding!

Citations:
[1] https://www.youtube.com/watch?v=PjeJm6oHWdo
[2] https://www.firstinspires.org/resource-library/ftc/technology-information-and-resources
[3] https://docs.wpilib.org/en/stable/docs/software/driverstation/driver-station-best-practices.html
[4] https://www.instructables.com/Programming-an-FRC-Robot/
[5] https://www.youtube.com/watch?v=3Q4nUHTWd48
[6] https://www.chiefdelphi.com/t/frc-programming-course-share-your-methods/475292
[7] https://www.youtube.com/watch?v=C5DqnIu6g8k
[8] https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html
[9] https://frcteam3255.github.io/FRC-Java-Tutorial/setup/install_software.html