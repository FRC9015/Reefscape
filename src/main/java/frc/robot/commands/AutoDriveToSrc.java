package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class AutoDriveToSrc extends Command {

  private final Drive drive;
  private final Pose2d sourcePose = Constants.FieldConstants.SourceL;
  private boolean atSetpoint = false;

  // PID controllers for x, y, and rotation
  private final PIDController xController =
      new PIDController(1.5, 0.0, 0.02); // Tune gains as needed
  private final PIDController yController =
      new PIDController(1.5, 0.0, 0.02); // Tune gains as needed
  private final PIDController rotationController =
      new PIDController(4, 0.0, 0.02); // Tune gains as needed

  public AutoDriveToSrc(Drive drive) {
    this.drive = drive;
    addRequirements(drive);

    // Set tolerances for PID controllers
    xController.setTolerance(0.1); // 10 cm tolerance
    yController.setTolerance(0.1); // 10 cm tolerance
    rotationController.setTolerance(Math.toRadians(2)); // 2 degrees tolerance
  }

  @Override
  public void initialize() {
    atSetpoint = false;

    // Reset PID controllers
    xController.reset();
    yController.reset();
    rotationController.reset();
  }

  @Override
  public boolean isFinished() {
    return atSetpoint;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  /**
   * Updates the target pose dynamically and calculates a motion plan to reach the source pose. This
   * method ensures the robot can navigate efficiently from its current position to the desired
   * source pose.
   */
  private void updateMotionPlan() {
    Pose2d currentPose = drive.getPose();

    // Calculate a direct path to the source pose
    // Twist2d error = currentPose.log(sourcePose);

    // Use a simple motion planning algorithm to adjust velocities dynamically
    // double distanceToTarget = Math.hypot(error.dx, error.dy);
    // double angleToTarget = Math.atan2(error.dy, error.dx);

    // Adjust velocities based on distance and angle
    double xVelocity =
        MathUtil.clamp(xController.calculate(currentPose.getX(), sourcePose.getX()), -3.0, 3.0);
    double yVelocity =
        MathUtil.clamp(yController.calculate(currentPose.getY(), sourcePose.getY()), -3.0, 3.0);
    double rotationalVelocity =
        MathUtil.clamp(
            rotationController.calculate(
                currentPose.getRotation().getRadians(), sourcePose.getRotation().getRadians()),
            -Math.PI,
            Math.PI);

    // Create robot-relative speeds
    ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds(xVelocity, yVelocity, rotationalVelocity);

    // Convert to field-relative speeds
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(robotRelativeSpeeds, currentPose.getRotation());

    // Command the drive subsystem to move
    drive.runVelocity(fieldRelativeSpeeds);

    // Check if the robot is near the target pose
    atSetpoint =
        xController.atSetpoint() && yController.atSetpoint() && rotationController.atSetpoint();
  }

  @Override
  public void execute() {
    // Continuously update the motion plan to adapt to the robot's current position
    Pose2d currentPose = drive.getPose();

    // Calculate the error as a Twist2d
    Twist2d error = currentPose.log(sourcePose);

    // Calculate velocities using PID controllers
    double xVelocity = MathUtil.clamp(xController.calculate(error.dx, 0.0), -3.0, 3.0);
    double yVelocity = MathUtil.clamp(yController.calculate(error.dy, 0.0), -3.0, 3.0);
    double rotationalVelocity =
        MathUtil.clamp(rotationController.calculate(error.dtheta, 0.0), -Math.PI, Math.PI);

    // Create robot-relative speeds
    ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds(xVelocity, yVelocity, rotationalVelocity);

    // Convert to field-relative speeds
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(robotRelativeSpeeds, currentPose.getRotation());

    // Command the drive subsystem to move
    drive.runVelocity(fieldRelativeSpeeds);

    // Check if the robot is near the target pose
    atSetpoint =
        xController.atSetpoint() && yController.atSetpoint() && rotationController.atSetpoint();
    updateMotionPlan();
  }
}
