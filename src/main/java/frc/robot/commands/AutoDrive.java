package frc.robot.commands;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.PhoenixUtil;
import org.littletonrobotics.junction.Logger;

public class AutoDrive extends Command {

  private PIDController rotationController = new PIDController(4, 0, 0.02); // 4 0.02
  private PIDController yController = new PIDController(1.5, 0.0, 0.02); // 4 0.04
  private PIDController xController = new PIDController(1.5, 0.0, 0.02); // 2.5 0.04
  private Pose2d targetPose, flippedPose;
  private Drive drive;
  public static boolean atSetpoint = false;

  public AutoDrive(Pose2d desiredPose, Drive drive) {
    this.drive = drive;
    flippedPose = FlippingUtil.flipFieldPose(desiredPose);
    this.targetPose = PhoenixUtil.isRed() ? flippedPose : desiredPose;
  }

  @Override
  public void initialize() {
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(Units.degreesToRadians(1));
    yController.setTolerance(Units.inchesToMeters(0.4));
    xController.setTolerance(Units.inchesToMeters(0.4));
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();

    // Calculate the error as a Twist2d
    Twist2d error = currentPose.log(targetPose);

    // Use the error to calculate velocities
    double xVelocity = xController.calculate(0, error.dx);
    double yVelocity = yController.calculate(0, error.dy);
    double rotationalVelocity = rotationController.calculate(0, error.dtheta);

    // Create robot-relative speeds
    ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
    robotRelativeSpeeds.vxMetersPerSecond = xVelocity;
    robotRelativeSpeeds.vyMetersPerSecond = yVelocity;
    robotRelativeSpeeds.omegaRadiansPerSecond = rotationalVelocity;

    // Convert to field-relative speeds
    ChassisSpeeds field =
        new ChassisSpeeds().fromFieldRelativeSpeeds(robotRelativeSpeeds, currentPose.getRotation());
    drive.runVelocity(field);

    // Log outputs
    Logger.recordOutput("AutoDrive/targetPose", targetPose);
    Logger.recordOutput("AutoDrive/robotrelativespeeds", field);
    Logger.recordOutput("AutoDrive/flippedPose", flippedPose);
    Logger.recordOutput("AutoDrive/error", error);
  }

  @Override
  public boolean isFinished() {
    atSetpoint = true;
    return rotationController.atSetpoint() && yController.atSetpoint() && xController.atSetpoint();
  }
}
