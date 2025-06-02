package frc.robot.commands;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer; // <-- Add this at the top
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AutoDrive extends Command {

  private PIDController rotationController = new PIDController(4, 0.01, 0.03);
  private PIDController yController = new PIDController(2.5, 0.05, 0.03);
  private PIDController xController = new PIDController(2.5, 0.05, 0.03);
  private Supplier<Pose2d> targetPose;
  private Pose2d targetPose2d, currentPose;
  private Drive drive;
  private Supplier<DriverStation.Alliance> allaince;

  private final Timer timer = new Timer(); // <-- Add this

  public AutoDrive(
      Supplier<Pose2d> desiredPose, Drive drive, Supplier<DriverStation.Alliance> alliance) {
    this.drive = drive;
    this.targetPose = desiredPose;
    this.allaince = alliance;
  }

  @Override
  public void initialize() {
    this.targetPose2d =
        allaince.get() == DriverStation.Alliance.Red
            ? FlippingUtil.flipFieldPose(targetPose.get())
            : targetPose.get();
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(Units.degreesToRadians(1));
    yController.setTolerance(Units.inchesToMeters(2.5));
    xController.setTolerance(Units.inchesToMeters(2.5));
    Logger.recordOutput("AutoDrive/alliance?", allaince.get());
    // Logger.recordOutput("AutoDrive/rotationController_atSetpoint", false);
    // Logger.recordOutput("AutoDrive/yController_atSetpoint", false);
    // Logger.recordOutput("AutoDrive/xController_atSetpoint", false);
    // Logger.recordOutput("AutoDrive/rotationController_Error", 0);
    // Logger.recordOutput("AutoDrive/yController_Error", 0);
    // Logger.recordOutput("AutoDrive/xController_Error", 0);

    timer.reset(); // <-- Start of timer setup
    timer.start(); // <--
  }

  @Override
  public void execute() {
    currentPose = drive.getPose();

    double rotationalVelocity =
        rotationController.calculate(
            currentPose.getRotation().getRadians(), targetPose2d.getRotation().getRadians());
    double yVelocity = yController.calculate(currentPose.getY(), targetPose2d.getY());
    double xVelocity = xController.calculate(currentPose.getX(), targetPose2d.getX());

    ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();

    robotRelativeSpeeds.vxMetersPerSecond = xVelocity;
    robotRelativeSpeeds.vyMetersPerSecond = yVelocity;
    robotRelativeSpeeds.omegaRadiansPerSecond = rotationalVelocity;

    ChassisSpeeds field =
        ChassisSpeeds.fromFieldRelativeSpeeds(robotRelativeSpeeds, currentPose.getRotation());
    drive.runVelocity(field);

    Logger.recordOutput("AutoDrive/currentPose", currentPose);
    Logger.recordOutput("AutoDrive/targetPose", targetPose2d);
    Logger.recordOutput("AutoDrive/robotrelativespeeds", field);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop(); // <-- Clean up the timer
  }

  @Override
  public boolean isFinished() {
    Logger.recordOutput("AutoDrive/rotationController_atSetpoint", rotationController.atSetpoint());
    Logger.recordOutput("AutoDrive/yController_atSetpoint", yController.atSetpoint());
    Logger.recordOutput("AutoDrive/xController_atSetpoint", xController.atSetpoint());
    Logger.recordOutput(
        "AutoDrive/rotationController_Error",
        currentPose.getRotation().getDegrees() - targetPose2d.getRotation().getDegrees());
    Logger.recordOutput(
        "AutoDrive/yController_Error",
        Units.metersToInches(currentPose.getY() - targetPose2d.getY()));
    Logger.recordOutput(
        "AutoDrive/xController_Error",
        Units.metersToInches(currentPose.getX() - targetPose2d.getX()));
    return (rotationController.atSetpoint() && yController.atSetpoint() && xController.atSetpoint())
        || timer.hasElapsed(3.5); // <-- 3.5-second timeout
  }
}
