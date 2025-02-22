package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.intake.Intake;

import java.util.Arrays;
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

  // Purpose: Warmup motorized subsystems (intake, endeffector) while pathifinding to an auto start
  // pose.
  public static Command pathfindToAutoStartPoseWhileWarmup(
      String desiredAuto, EndEffector endEffector, Intake intake) {
    Pose2d startPose = returnFirstPathinAuto(desiredAuto).getStartingHolonomicPose().get();

    return new ParallelCommandGroup(
        AutoBuilder.pathfindToPose(
            startPose,
            AutoConstants.PP_CONSTRAINTS,
            returnFirstPathinAuto(desiredAuto).getGoalEndState().velocityMPS()),
        Commands.runOnce(() -> endEffector.setRPM(6000), endEffector),
        Commands.runOnce(() -> intake.setRPM(6000), intake));
  }

  private Pose2d findClosestReefPose(Pose2d currentPose) {
    Pose2d[] reefPoses = {
      Constants.FieldConstants.REEF_A, 
      Constants.FieldConstants.REEF_B, 
      Constants.FieldConstants.REEF_C, 
      Constants.FieldConstants.REEF_D, 
      Constants.FieldConstants.REEF_E, 
      Constants.FieldConstants.REEF_F};
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

public boolean isNearReef(Drive drive) {
  Pose2d currentPose = drive.getPose();
  Pose2d closestReefPose = findClosestReefPose(currentPose);
  
  // Check if the robot is within 2 meters of the closest reef pose
  boolean isWithinTwoMeters = currentPose.getTranslation().getDistance(closestReefPose.getTranslation()) <= 2.0;
 
  // Return true if both conditions are met
  return isWithinTwoMeters;
}

  
  public Command getReefPathCommand(Drive drive, Elevator elevator, EndEffector endEffector) {
    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = findClosestReefPose(currentPose);
    Trigger isBotWithinReefPerimeter = new Trigger(()-> isNearReef(drive));
    return new SequentialCommandGroup(
        AutoBuilder.pathfindToPose(targetPose, Constants.AutoConstants.PP_CONSTRAINTS)

    );
}

}
