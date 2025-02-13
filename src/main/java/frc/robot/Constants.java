// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  // public static final Transform3d CAMERA_1_TO_ROBOT = new Transform3d();

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public class CameraConstants {
    public static final String CAMERA_2_NAME = "Camera2";
    public static final String CAMERA_1_NAME = "Camera1";
    // Camera positions relative to the robot's center in meters
    public static final Transform3d CAMERA_1_TO_ROBOT =
        new Transform3d(
            0.3,
            0.0,
            0.2,
            new Rotation3d(0, 0, Math.PI) // Sample metrics for front-mounted camera in meters
            );

    public static final Transform3d CAMERA_2_TO_ROBOT =
        new Transform3d(
            -0.3,
            0.0,
            0.2,
            new Rotation3d(0, 0, 0) // Sample metrics for rear-mounted camera in meters
            );
  }
}
