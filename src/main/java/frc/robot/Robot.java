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

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The main robot class, extending {@link LoggedRobot}. This class is configured to run
 * automatically and execute the appropriate methods for each robot mode.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  /** Constructs the Robot instance and initializes logging and configuration checks. */
  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncommitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
      default:
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Check for valid swerve configuration
    var modules =
        new SwerveModuleConstants[] {
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight
        };

    for (var constants : modules) {
      if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
          || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
        throw new RuntimeException(
            "You are using an unsupported swerve configuration, which this template does not "
                + "support without manual customization. The 2025 release of Phoenix supports "
                + "some swerve configurations which were not available during 2025 beta testing, "
                + "preventing any development and support from the AdvantageKit developers.");
      }
    }

    // Instantiate our RobotContainer. This sets up button bindings and dashboard configurations.
    robotContainer = new RobotContainer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 99);

    // Runs the Command Scheduler
    CommandScheduler.getInstance().run();

    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);

    Logger.recordOutput("Robopose", new Pose2d());
    Logger.recordOutput(
        "ZeroedComponentPoses",
        new Pose3d[] {new Pose3d(), new Pose3d()} // Two components, both initially at zero
        );
    double totalExtension = -0.45 * Math.cos(Math.PI * Timer.getFPGATimestamp() / 2) + 0.5;
    // Clamp stage 1 to 0.5. When totalExtension is below 0.5, stage 1 follows it.
    double stage1Extension = Math.min(totalExtension, 0.5);
    // The carriage (stage 2) only moves once stage1Extension has hit 0.5.
    double stage2Extension = (totalExtension > 0.5) ? totalExtension - 0.5 : 0.0;

    // Now, the absolute position of stage 1 is stage1Extension,
    // and the absolute position of the carriage is stage1Extension + stage2Extension,
    // which is equal to totalExtension.
    Logger.recordOutput(
        "FinalComponentPoses",
        new Pose3d[] {
          // Lower stage’s absolute pose
          new Pose3d(0.0, 0.0, stage1Extension + 0.03, new Rotation3d(0.0, 0.0, 0.0)),
          // Carriage’s absolute pose (mounted on stage 1)
          new Pose3d(
              0.0, 0.0, stage1Extension + stage2Extension - 0.1, new Rotation3d(0.0, 0.0, 0.0))
        });
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once at the start of the autonomous period. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // Schedule the autonomous command if available
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during the autonomous period. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once at the start of the teleoperated period. */
  @Override
  public void teleopInit() {
    // Ensures autonomous command stops when teleop starts
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /**
   * This function is called once when the robot enters simulation mode. Initializes the
   * SimulatedArena **only** if running in simulation mode.
   */
  @Override
  public void simulationInit() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      RobotContainer.setBargePose();
      SimulatedArena.getInstance().simulationPeriodic();
      // Add a Reescape Algae to the field
      SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(3, 3)));
      SimulatedArena.getInstance()
          .addGamePiece(new ReefscapeCoralOnField(new Pose2d(4, 4, new Rotation2d())));
    }
  }

  /**
   * This function is called periodically during simulation mode. Runs the SimulatedArena **only**
   * if running in simulation mode.
   */
  @Override
  public void simulationPeriodic() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      SimulatedArena.getInstance().simulationPeriodic();
    }
  }

  void periodic() {
    // Get the positions of the algae (both on the field and in the air)
    Pose3d[] algaePoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Algae");
    // Publish to telemetry using AdvantageKit
    Logger.recordOutput("FieldSimulation/AlgaePositions", algaePoses);
    Pose3d[] coralPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Coral");
    Logger.recordOutput("FieldSimulation/CoralPositions", coralPoses);
  }
}
