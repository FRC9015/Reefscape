package frc.robot.commands;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.photon.Vision;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * AutoDrive command drives the robot from its current pose to a target pose using PID control.
 * It integrates odometry, vision, and collision detection for robust autonomous driving.
 */
public class AutoDrive extends Command {
  // PID controllers for rotation and translational control
  private final PIDController rotationController = new PIDController(4, 0, 0.02);
  private final PIDController yController = new PIDController(2.5, 0.0, 0.03);
  private final PIDController xController = new PIDController(2.5, 0.0, 0.03);

  // Target pose and flipped pose for Red alliance
  private Pose2d targetPose, flippedPose;

  // Subsystem references
  private final Drive drive;
  private final Vision vision;
  private final Supplier<DriverStation.Alliance> allianceSupplier;

  // Timing variables
  private double lastUpdateTime = 0;

  // Collision detection variables
  private double lastLinearAccel = 0;
  private boolean recoveringFromCollision = false;
  private double recoveryStartTime = 0;

  // Public flag for diagnostics
  public static boolean atSetpoint = false;

  /**
   * Constructs the AutoDrive command.
   *
   * @param desiredPose The desired target pose.
   * @param drive The Drive subsystem.
   * @param vision The Vision subsystem.
   * @param allianceSupplier Supplier for alliance information.
   */
  public AutoDrive(
      Pose2d desiredPose, Drive drive, Vision vision, Supplier<DriverStation.Alliance> allianceSupplier) {
    this.drive = drive;
    this.vision = vision;
    this.allianceSupplier = allianceSupplier;

    // Flip the desired pose for Red alliance
    this.flippedPose = FlippingUtil.flipFieldPose(desiredPose);
    this.targetPose = desiredPose;

    // Declare subsystem dependency
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Determine the target pose based on alliance color
    this.targetPose = allianceSupplier.get() == DriverStation.Alliance.Red ? flippedPose : targetPose;

    // Configure PID controllers
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
    rotationController.setTolerance(Units.degreesToRadians(1));
    yController.setTolerance(Units.inchesToMeters(2));
    xController.setTolerance(Units.inchesToMeters(2));

    // Reset timing and collision state
    lastUpdateTime = Timer.getFPGATimestamp();
    recoveringFromCollision = false;

    // Log the target pose
    Logger.recordOutput("AutoDrive/TargetPose", targetPose);
  }

  @Override
  public void execute() {
    double currentTime = Timer.getFPGATimestamp();
    double dt = currentTime - lastUpdateTime; // Time delta for calculations
    lastUpdateTime = currentTime;

    Pose2d currentPose = drive.getPose(); // Get the current robot pose
    Twist2d error = calculateBoundedError(currentPose, targetPose);

    if (!recoveringFromCollision) {
      // Validate pose using vision
      if (!validatePose()) {
        Logger.recordOutput("AutoDrive/PoseValidationFailed", true);
        return; // Skip execution if pose validation fails
      }

      // Adjust rotation PID gain dynamically
      updateOrientationCorrection(error);

      // Calculate chassis speeds using adaptive PID control
      ChassisSpeeds speeds = calculateAdaptiveSpeeds(error, dt);

      // Check for collisions and handle recovery if needed
      if (detectCollision()) {
        handleCollisionRecovery();
      }

      // Command the drivetrain with calculated speeds
      drive.runVelocity(speeds);

      // Log diagnostics
      logDiagnostics(currentPose, error, speeds);
    } else if (Timer.getFPGATimestamp() - recoveryStartTime > 0.5) {
      // Resume normal operation after recovery period
      recoveringFromCollision = false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain when the command ends
    drive.stop();
    Logger.recordOutput("AutoDrive/Finished", true);
  }

  @Override
  public boolean isFinished() {
    // Check if all PID controllers are at their setpoints
    atSetpoint =
        rotationController.atSetpoint() && yController.atSetpoint() && xController.atSetpoint();

    // Validate the robot's pose against stricter tolerances
    Pose2d currentPose = drive.getPose();
    Twist2d error = calculateBoundedError(currentPose, targetPose);

    boolean positionValid =
        Math.abs(error.dx) < Units.inchesToMeters(1)
            && Math.abs(error.dy) < Units.inchesToMeters(1);
    boolean rotationValid = Math.abs(error.dtheta) < Units.degreesToRadians(1);

    return atSetpoint && positionValid && rotationValid;
  }

  /**
   * Calculates the error between the current and target pose, wrapping angular error to [-π, π].
   *
   * @param currentPose The current robot pose.
   * @param targetPose The desired target pose.
   * @return The pose error as a Twist2d.
   */
  private Twist2d calculateBoundedError(Pose2d currentPose, Pose2d targetPose) {
    double dx = targetPose.getX() - currentPose.getX();
    double dy = targetPose.getY() - currentPose.getY();
    double dtheta = targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians();
    dtheta = Math.atan2(Math.sin(dtheta), Math.cos(dtheta)); // Wrap angle error
    return new Twist2d(dx, dy, dtheta);
  }

  /**
   * Dynamically adjusts the proportional gain for rotation based on angular error.
   *
   * @param error The current pose error.
   */
  private void updateOrientationCorrection(Twist2d error) {
    rotationController.setP(Math.abs(error.dtheta) > Units.degreesToRadians(10) ? 4 : 2);
  }

  /**
   * Calculates chassis speeds using adaptive PID control.
   *
   * @param error The pose error.
   * @param dt The time delta since the last update.
   * @return The field-relative chassis speeds.
   */
  private ChassisSpeeds calculateAdaptiveSpeeds(Twist2d error, double dt) {
    Rotation2d orientationError = targetPose.getRotation().minus(drive.getPose().getRotation());

    double xVel =
        xController.calculate(0, error.dx) + orientationError.getCos() * error.dx * dt * 0.4;
    double yVel =
        yController.calculate(0, error.dy) + orientationError.getSin() * error.dy * dt * 0.4;
    double rotVel = rotationController.calculate(0, error.dtheta);

    return ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, rotVel, drive.getPose().getRotation());
  }

  /**
   * Detects collisions based on sudden changes in acceleration.
   *
   * @return true if a collision is detected, false otherwise.
   */
  private boolean detectCollision() {
    ChassisSpeeds currentSpeeds = drive.getChassisSpeeds();
    double currentLinearAccel = Math.sqrt(
        Math.pow(currentSpeeds.vxMetersPerSecond, 2) + Math.pow(currentSpeeds.vyMetersPerSecond, 2)
    );

    double jerk = (currentLinearAccel - lastLinearAccel) / (Timer.getFPGATimestamp() - lastUpdateTime);
    lastLinearAccel = currentLinearAccel;

    final double COLLISION_JERK_THRESHOLD = 10.0; // Adjust based on testing
    return Math.abs(jerk) > COLLISION_JERK_THRESHOLD;
  }

  /**
   * Handles collision recovery by resetting PID controllers and fusing vision and odometry data.
   */
  private void handleCollisionRecovery() {
    recoveringFromCollision = true;
    recoveryStartTime = Timer.getFPGATimestamp();

    // Reset PID controllers
    rotationController.reset();
    xController.reset();
    yController.reset();

    // Use the fused pose from odometry
    Pose2d fusedPose = drive.getPose();
    drive.setPose(fusedPose);

    Logger.recordOutput("AutoDrive/CollisionDetected", true);
  }

  /**
   * Validates the robot's pose using vision data.
   *
   * @return true if the pose is valid, false otherwise.
   */
  private boolean validatePose() {
    Pose2d visionPose = vision.getLatestPose2d(1); // Assuming camera 1 is used

    if (visionPose == null) {
      return true; // Skip validation if vision data is unavailable
    }

    Twist2d error = calculateBoundedError(visionPose, targetPose);

    boolean positionValid =
        Math.abs(error.dx) < Units.inchesToMeters(1)
            && Math.abs(error.dy) < Units.inchesToMeters(1);
    boolean rotationValid = Math.abs(error.dtheta) < Units.degreesToRadians(1);

    return positionValid && rotationValid;
  }

  /**
   * Logs diagnostic data for troubleshooting and tuning.
   *
   * @param currentPose The current robot pose.
   * @param error The pose error.
   * @param speeds The commanded chassis speeds.
   */
  private void logDiagnostics(Pose2d currentPose, Twist2d error, ChassisSpeeds speeds) {
    Logger.recordOutput("AutoDrive/TargetPose", targetPose);
    Logger.recordOutput("AutoDrive/CurrentSpeeds", speeds);
    Logger.recordOutput("AutoDrive/PoseError", error);
  }
}
