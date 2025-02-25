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

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final double SLOW_MODE_CONSTANT = 0.55;
  public static final double ANTI_CAPSIZE_CONSTANT = 0.275;

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

  public static class FieldConstants {
    public static final Pose2d reef_AL1 =
        new Pose2d(new Translation2d(3.25, 3.8), new Rotation2d());
    public static final Pose2d bargeFar = new Pose2d(new Translation2d(7.6, 6.6), new Rotation2d());
    public static final Pose2d bargeNear = new Pose2d(new Translation2d(7.6, 5), new Rotation2d());

    // Reef poses/locations
    public static final Pose2d REEF_A =
        new Pose2d(new Translation2d(3.033, 4.103), new Rotation2d());
    public static final Pose2d REEF_B =
        new Pose2d(new Translation2d(3.261, 5.230), new Rotation2d());
    public static final Pose2d REEF_C =
        new Pose2d(new Translation2d(5.287, 5.386), new Rotation2d());

    public static final Pose2d REEF_D = new Pose2d(new Translation2d(6, 4.103), new Rotation2d());
    public static final Pose2d REEF_E =
        new Pose2d(new Translation2d(5.632, 2.832), new Rotation2d());
    public static final Pose2d REEF_F =
        new Pose2d(new Translation2d(3.836, 2.616), new Rotation2d());
  }

  public static class AutoConstants {
    public static final PathConstraints PP_CONSTRAINTS =
        new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
  }
}
