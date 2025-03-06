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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always
 * "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics
 * sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    public static final double SLOW_MODE_CONSTANT = 0.55;

    // public static final Transform3d CAMERA_1_TO_ROBOT = new Transform3d();

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static class CameraConstants {
        public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        public static final Transform3d bowPose = new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(15), Units.inchesToMeters(0), Units.inchesToMeters(7)),
                new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(0)));

        public static final Transform3d starboardPose = new Transform3d(
                new Translation3d(
                        Units.inchesToMeters(0), -Units.inchesToMeters(15), Units.inchesToMeters(7)),
                new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(270)));

        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static class FieldConstants {
        public static final Pose2d reef_AL1 = new Pose2d(new Translation2d(3.25, 3.8), new Rotation2d());
        public static final Pose2d bargeFar = new Pose2d(new Translation2d(7.6, 6.6), new Rotation2d());
        public static final Pose2d bargeNear = new Pose2d(new Translation2d(7.6, 5), new Rotation2d());
        public static final Pose2d bargeMid = new Pose2d(new Translation2d(7.137, 6.136), new Rotation2d(Math.PI));
        public static final Pose2d SourceL = new Pose2d(
                new Translation2d(1.403, 6.991), new Rotation2d(Radians.convertFrom(-55, Degrees)));
        // Reef poses/locations
        public static final Pose2d REEF_AR = new Pose2d(new Translation2d(3.104, 3.865), new Rotation2d());
        public static final Pose2d REEF_AL = new Pose2d(new Translation2d(3.119, 4.215), new Rotation2d());

        public static final Pose2d REEF_BR = new Pose2d(new Translation2d(3.723, 5.126),
                new Rotation2d(5 * Math.PI / 3));
        public static final Pose2d REEF_BL = new Pose2d(new Translation2d(3.977, 5.335),
                new Rotation2d(5 * Math.PI / 3));

        public static final Pose2d REEF_CR = new Pose2d(new Translation2d(4.992, 5.263),
                new Rotation2d(4 * Math.PI / 3));
        // everything under this comment is not proper pose above is proper pose
        public static final Pose2d REEF_CL = new Pose2d(new Translation2d(5.287, 5.386),
                new Rotation2d(4 * Math.PI / 3));

        public static final Pose2d REEF_DR = new Pose2d(new Translation2d(6, 4.103), new Rotation2d());
        public static final Pose2d REEF_DL = new Pose2d(new Translation2d(6, 4.103), new Rotation2d());

        public static final Pose2d REEF_ER = new Pose2d(new Translation2d(5.632, 2.832), new Rotation2d());
        public static final Pose2d REEF_EL = new Pose2d(new Translation2d(5.632, 2.832), new Rotation2d());

        public static final Pose2d REEF_FR = new Pose2d(new Translation2d(3.836, 2.616), new Rotation2d());
        public static final Pose2d REEF_FL = new Pose2d(new Translation2d(3.836, 2.616), new Rotation2d());
    }

    public static class AutoConstants {
        public static final PathConstraints PP_CONSTRAINTS = new PathConstraints(3.0, 4.0, Units.degreesToRadians(540),
                Units.degreesToRadians(720));
    }

    public static class MotorIDConstants {
        public static final int END_EFFECTOR_MOTOR_ID = 2;
        public static final int INTAKE_MOTOR_ID = 1;
        public static final int PIVOT_MOTOR_ID = 3;
        public static final int ALGAE_MOTOR_ID = 4;

        public static final int ELEVATOR_MOTOR_ID1 = 9;
        public static final int ELEVATOR_MOTOR_ID2 = 10;
        public static final int ELEVATOR_ENCODER_ID = 8;
        public static final double ELEVATOR_MAGNET_OFFSET = 0.15;
    }

    public static final class buttonBoxIOInputs {
        public static enum buttonBoxIds {
            REEF_AL(10),
            REEF_AR(12),
            REEF_BL(6),
            REEF_BR(8),
            REEF_CL(3),
            REEF_CR(5),
            REEF_DL(20),
            REEF_DR(1),
            REEF_EL(18),
            REEF_ER(22),
            REEF_FL(14),
            REEF_FR(16),
            ELEVATOR_L1(32),
            ELEVATOR_L2(30),
            ELEVATOR_L3(28),
            ELEVATOR_L4(26),
            ABORT(24);

            // Field to store the button ID
            private final int buttonID;

            // Constructor to assign the button ID
            buttonBoxIds(int buttonID) {
                this.buttonID = buttonID;
            }

            // Getter for button ID
            public int getButtonID() {
                return buttonID;
            }
        }
    }
}