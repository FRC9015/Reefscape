package frc.robot.commands;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AutoDrive extends Command {

  private PIDController rotationController = new PIDController(4, 0, 0.02); // 4 0.02
  private PIDController yController = new PIDController(2.5, 0.0, 0.03); // 4 0.04
  private PIDController xController = new PIDController(2.5, 0.0, 0.03); // 2.5 0.04
  private Pose2d targetPose, flippedPose;
  private Drive drive;
  private Supplier<DriverStation.Alliance> allaince;

  public AutoDrive(Pose2d desiredPose, Drive drive, Supplier<DriverStation.Alliance> alliance) {
    this.drive = drive;
    flippedPose = FlippingUtil.flipFieldPose(desiredPose);
    this.targetPose = desiredPose;
    this.allaince = alliance;
  }

  @Override
  public void initialize() {

    this.targetPose = allaince.get() == DriverStation.Alliance.Red ? flippedPose : targetPose;
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(Units.degreesToRadians(1));
    yController.setTolerance(Units.inchesToMeters(0.4));
    xController.setTolerance(Units.inchesToMeters(0.4));
    Logger.recordOutput("AutoDrive/alliance?", allaince.get());
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();

    double rotationalVelocity =
        rotationController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    double yVelocity = yController.calculate(currentPose.getY(), targetPose.getY());
    double xVelocity = xController.calculate(currentPose.getX(), targetPose.getX());

    ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();

    robotRelativeSpeeds.vxMetersPerSecond = xVelocity;
    robotRelativeSpeeds.vyMetersPerSecond = yVelocity;
    robotRelativeSpeeds.omegaRadiansPerSecond = rotationalVelocity;

    ChassisSpeeds field =
        new ChassisSpeeds().fromFieldRelativeSpeeds(robotRelativeSpeeds, currentPose.getRotation());
    drive.runVelocity(field);

    Logger.recordOutput("AutoDrive/targetPose", targetPose);
    Logger.recordOutput("AutoDrive/robotrelativespeeds", field);
    Logger.recordOutput("AutoDrive/flippedPose", flippedPose);
  }

  @Override
  public boolean isFinished() {

    return rotationController.atSetpoint() && yController.atSetpoint() && xController.atSetpoint();
  }
}
