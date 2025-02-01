package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.intake.Intake;

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
        path,
        new PathConstraints(3, 3, Math.PI / 2, Math.PI / 2)
    );
  }
  catch(Exception e) {
    return Commands.runOnce(() -> System.out.println("Path not found: " + pathName));
  }
  
}

public PathPlannerPath returnFirstPathinAuto(String autoName) {
        try {
            List<PathPlannerPath> pathCollection = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
            return pathCollection.get(0);
        }
        catch(Exception e) {
            System.out.println("Path not found: " + autoName + "Exception: " + e);
            return null;
        }
    }

public Command pathfindToAutoStartPoseWhileWarmup(Pose2d currentPose, String desiredAuto, EndEffector endEffector, Intake intake) {
        Pose2d startPose = returnFirstPathinAuto(desiredAuto).getStartingHolonomicPose().get();

        return new ParallelCommandGroup(
            AutoBuilder.pathfindToPose(startPose, AutoConstants.PP_CONSTRAINTS, returnFirstPathinAuto(desiredAuto).getIdealStartingState().velocityMPS()),
            Commands.runOnce(()->endEffector.setRPM(6000), endEffector),
            Commands.runOnce(()->intake.setRPM(6000), intake)
        );
    }

}