package frc.robot.commands;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs.ElevatorState;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AutoDrive extends Command {

  private PIDController rotationController = new PIDController(4, 0, 0.02);
  private PIDController yController = new PIDController(3, 0.0, 0.02);
  private PIDController xController = new PIDController(3, 0.0, 0.02);
  private Supplier<Pose2d> targetPose;
  private ElevatorState state;
  private Pose2d flippedPose, targetPose2d;
  private Drive drive;
  private Supplier<DriverStation.Alliance> allaince;

  private final Timer timer = new Timer();

  // For calculus-based settling
  private Pose2d previousPose;
  private double previousTime = 0.0;
  private double previousVelocity = 0.0;

  private final BooleanSupplier isSettlingQuickly =
      () -> {
        Pose2d currentPose = drive.getPredictedPose();
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - previousTime;

        if (dt == 0) return false;

        // Positional error magnitude (Euclidean distance)
        double dx = currentPose.getX() - targetPose2d.getX();
        double dy = currentPose.getY() - targetPose2d.getY();
        double error = Math.hypot(dx, dy);

        // Previous error magnitude
        Pose2d prev = previousPose == null ? currentPose : previousPose;
        double prevDx = prev.getX() - targetPose2d.getX();
        double prevDy = prev.getY() - targetPose2d.getY();
        double prevError = Math.hypot(prevDx, prevDy);

        double velocity = (error - prevError) / dt;
        double acceleration = (velocity - previousVelocity) / dt;

        // Update for next iteration
        previousPose = currentPose;
        previousTime = currentTime;
        previousVelocity = velocity;

        boolean closeEnough = error < Units.inchesToMeters(0.75);
        boolean slowEnough = Math.abs(velocity) < Units.inchesToMeters(0.5);
        boolean decelerating = acceleration < 0;

        Logger.recordOutput("AutoDrive/SettleError", error);
        Logger.recordOutput("AutoDrive/SettleVelocity", velocity);
        Logger.recordOutput("AutoDrive/SettleAccel", acceleration);

        return closeEnough && slowEnough && decelerating;
      };

  public AutoDrive(
      Supplier<Pose2d> desiredPose, Drive drive, Supplier<DriverStation.Alliance> alliance) {
    this.drive = drive;
    this.targetPose = desiredPose;
    this.allaince = alliance;
  }

  @Override
  public void initialize() {
    drive.setTargetPose(targetPose2d);
    flippedPose = FlippingUtil.flipFieldPose(targetPose.get());
    this.targetPose2d =
        allaince.get() == DriverStation.Alliance.Red ? flippedPose : targetPose.get();
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(Units.degreesToRadians(1));
    yController.setTolerance(Units.inchesToMeters(0.4));
    xController.setTolerance(Units.inchesToMeters(0.4));
    Logger.recordOutput("AutoDrive/alliance?", allaince.get());

    timer.reset();
    timer.start();

    previousTime = Timer.getFPGATimestamp();
    previousPose = drive.getPredictedPose();
    previousVelocity = 0.0;
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getPredictedPose();

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
        new ChassisSpeeds().fromFieldRelativeSpeeds(robotRelativeSpeeds, currentPose.getRotation());
    drive.runVelocity(field);

    Logger.recordOutput("AutoDrive/targetPose", targetPose.get());
    Logger.recordOutput("AutoDrive/robotrelativespeeds", field);
    Logger.recordOutput("AutoDrive/flippedPose", flippedPose);
    Logger.recordOutput("AutoDrive/state", state);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    boolean pidDone =
        rotationController.atSetpoint() && yController.atSetpoint() && xController.atSetpoint();
    boolean timeout = timer.hasElapsed(1.5);
    boolean settlingFast = isSettlingQuickly.getAsBoolean();

    return pidDone || settlingFast || timeout;
  }
}
