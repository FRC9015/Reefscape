**## SYSTEM ARCHETECTURE: 
Diagram (diagram here)
### Subsystems:
####    Drivetrain: move robot QUICK and PRECISE.
**Inputs:** *gyroscope* (e.g., NavX, Pigeon IMU) for orentation and angular velocity, **encoders** on motors for distance and speed.
**Output:** *motors* on driving wheels (Kraken-X50).
**Features:**
*    * *PID Control and SYSID Constants* for maintaining constant velocity, angular turning, and path-following and angular adjustments in a closed loop system
    * *feedforward control* used with PID for improved acceleration control during trajectory following
    * *Kinematics* for converting desired velocities into motor commands or supporting smooth omni or swerve drivetrain movements
    * *PathPlanner* Integration: for trajectory generation and real-time trajectory tracking for autonomous routine.
---
####    End Efficator: score Coral.
 **Inputs:**
*   *  *Encoder* Connected Status for precise control over motor motion.
    * *Applied voltage* for precise voltage control interface for varied speed outputs. 
    * *Current Amps* for monitoring end effector motor current, helpful for logging and control through current if needed.
    * *End Effector RP*: Derived from CANCoder Encoder, for control over exact RPM. 
    * *Coral Detected Boolean* for communicating game piece status with drivers and communicating with other subsystems to coordinate movement (e.g. when to raise elevator)
**Output:** *motor* push coral out using kraken motors
**Features:**
* *Coral Detection Sensor* tracks the movement of coral 
* *Simulation Input-Output (IO) Component* Implementing the end effector subsystem’s IO file functionality, the simulation file creates a virtual instance of a Kraken X60 motor
* *Talon FX Input-Output (IO) Component* Implementing the end effector subsystem’s IO file functionality, the Talon FX file creates a physical instance of Kraken X60 motor (both End Effector IOs have identical commands)
---
#### Vision: visual feedback for piece alignment and scoring accuracy.
**Inputs:**  *ArduCAM and PhotonVision:* Captures field elements and game piece locations.
**Outputs:**   Target coordinates for drivetrain and end effector adjustments.
**Features:**  
*  *Pose Estimation:* calculates robot position using camera data
*  *Target Tracking Algorithms:* implements algorithms like AprilTag detection or feature-based tracking to guide actions.
*  *Trajectory Adjustment:* uses the vision system to correct drift or misalignment during movement.
---
#### climber: for climbing the deep cage
**Input:** *Encoders:* Provide precise control over climbing motion.
**Output:** *Motors and Hinges:* Drive the climbing and stabilization mechanisms.
**Features:**
* *PID + Feedforward Control:* for smooth motor motion and stable positioning during the climb.
* *Dynamic Load Compensation:* adjusts motor output based on the weight distribution and climb angle.
* *Pre-Climb Alignment System:* uses sensor feedback to ensure accurate placement before initiating the climb.
--- 
