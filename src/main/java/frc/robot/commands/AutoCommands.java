package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.intake.Intake;
import java.util.List;

public class AutoCommands {

  // Constants for below commands

  public static PathConstraints constraints =
      new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  public static Command resetPosetoStartofPath(Pose2d pose, PathPlannerPath path, Drive drive) {
    return Commands.runOnce(() -> drive.setPose((path.getStartingHolonomicPose().get())), drive);
  }

  public static Command pathOnTheFlytoPose(Pose2d targetPose) {

    Command pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
    return pathfindingCommand;
  }

  public static Command correcToDesiredPath(PathPlannerPath desiredPath) {
    Command correctToDesiredPath = AutoBuilder.pathfindThenFollowPath(desiredPath, constraints);
    return correctToDesiredPath;
  }

  public Command pathfindThenFollowPath(String pathName) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

      return AutoBuilder.pathfindThenFollowPath(
          path, new PathConstraints(3, 3, Math.PI / 2, Math.PI / 2));
    } catch (Exception e) {
      return Commands.runOnce(() -> System.out.println("Path not found: " + pathName));
    }
  }

  public static PathPlannerPath returnFirstPathinAuto(String autoName) {
    try {
      List<PathPlannerPath> pathCollection = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
      return pathCollection.get(0);
    } catch (Exception e) {
      System.out.println("Path not found: " + autoName + "Exception: " + e);
      return null;
    }
  }

  // Purpose: Warmup motorized subsystems (intake, endeffector) while pathifinding to an auto
  // pose.
  public static Command pathfindToAutoStartPoseWhileWarmup(
      String desiredAuto, EndEffector endEffector, Intake intake) {
    Pose2d startPose = returnFirstPathinAuto(desiredAuto).getStartingHolonomicPose().get();

    return new ParallelCommandGroup(
        AutoBuilder.pathfindToPose(
            startPose,
            AutoConstants.PP_CONSTRAINTS,
            returnFirstPathinAuto(desiredAuto).getGoalEndState().velocityMPS()),
        Commands.runOnce(() -> endEffector.setVoltage(6), endEffector),
        Commands.runOnce(() -> intake.setRPM(6000), intake));
  }

  /**
   * Finds the closest reef pose to the current robot position.
   *
   * @param currentPose The current pose of the robot.
   * @return The Pose2d of the closest reef.
   */
  private Pose2d findClosestReefPose(Pose2d currentPose) {
    Pose2d[] reefPoses = {
      Constants.FieldConstants.REEF_AR,
      Constants.FieldConstants.REEF_BR,
      Constants.FieldConstants.REEF_CR,
      Constants.FieldConstants.REEF_DR,
      Constants.FieldConstants.REEF_ER,
      Constants.FieldConstants.REEF_FR,
      Constants.FieldConstants.REEF_AL,
      Constants.FieldConstants.REEF_BL,
      Constants.FieldConstants.REEF_CL,
      Constants.FieldConstants.REEF_DL,
      Constants.FieldConstants.REEF_EL,
      Constants.FieldConstants.REEF_FL
    };
    Pose2d closestPose = reefPoses[0];
    double minDistance = Double.MAX_VALUE;

    for (Pose2d reefPose : reefPoses) {
      double distance = currentPose.getTranslation().getDistance(reefPose.getTranslation());
      if (distance < minDistance) {
        minDistance = distance;
        closestPose = reefPose;
      }
    }

    return closestPose;
  }

  /**
   * Checks if the robot is within 2 meters of the nearest reef pose.
   *
   * @param drive The Drive subsystem to get the current robot pose.
   * @return true if the robot is within 2 meters of the nearest reef, false otherwise.
   */

  // (THIS BOOLEAN UPDATER MUST RUN PERIODICALLY TO SERVE ITS INTENDED PURPOSE)
  public boolean isNearReef(Drive drive) {
    boolean isWithinTwoMeters = false;
    Pose2d currentPose = drive.getPose();
    Pose2d closestReefPose = findClosestReefPose(currentPose);

    // Check if the robot is within 2 meters of the closest reef pose
    isWithinTwoMeters =
        currentPose.getTranslation().getDistance(closestReefPose.getTranslation()) <= 2.0;

    // Return true if both conditions are met
    return isWithinTwoMeters;
  }

  /**
   * Checks if the robot is near a reef and executes elevator and end effector commands if true.
   *
   * @param desiredElevatorState The desired state for the elevator.
   * @param drive The Drive subsystem.
   * @param elevator The Elevator subsystem.
   * @param endEffector The EndEffector subsystem.
   * @return A Command to execute elevator preset and run end effector if near reef, or an empty
   *     command if not.
   */
  public Command check(
      ElevatorIOInputs.ElevatorState desiredElevatorState,
      Drive drive,
      Elevator elevator,
      EndEffector endEffector) {
    boolean isBotWithinReefPerimeter = isNearReef(drive);
    if (isBotWithinReefPerimeter) {
      return new ParallelCommandGroup(
          elevator.executePreset(desiredElevatorState), endEffector.runEffector(3000));
    }

    System.out.println("Robot not sufficiently within reef location.");
    return new InstantCommand();
  }

  /**
   * Creates a command to pathfind to the nearest reef pose and execute scoring actions.
   *
   * @param drive The Drive subsystem.
   * @param elevator The Elevator subsystem.
   * @param endEffector The EndEffector subsystem.
   * @param desiredState The desired state for the elevator.
   * @return A Command that combines pathfinding to the nearest reef and checking for scoring
   *     conditions.
   */
  public Command getReefPathCommand(
      Drive drive,
      Elevator elevator,
      EndEffector endEffector,
      ElevatorIOInputs.ElevatorState desiredState) {
    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = findClosestReefPose(currentPose);
    return new ParallelCommandGroup(
        drive.pathfindToPoseFlipped(targetPose, 0),
        check(desiredState, drive, elevator, endEffector));
  }
}
