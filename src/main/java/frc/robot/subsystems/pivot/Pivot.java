package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.pivot.PivotIO.PivotIOInputs;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private final Alert encoderDisconnectedAlert;
  private final PIDController pidController;

  // Pivot PID constants
  private static final double kP = 10.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kToleranceMeters = 0.01; // Acceptable position error in meters

  public Pivot(PivotIO io) {
    this.io = io;
    this.pidController = new PIDController(kP, kI, kD);
    this.pidController.setTolerance(kToleranceMeters);
    encoderDisconnectedAlert = new Alert("Pivot encoder disconnected!", AlertType.kError);
  }

  /** Periodically updates the algae's state and logs inputs. */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);

    // Keep updating the alert
    encoderDisconnectedAlert.set(!inputs.pivotEncoderConnected);
  }

  /**
   * Sets the pivot to the specified preset state using PID control.
   *
   * @param state The desired preset state.
   */
  public void setPreset(PivotIOInputs.PivotPositions state) {
    double targetPosition = state.getPivotPosition();
    pidController.setSetpoint(targetPosition);
    double output = pidController.calculate(inputs.pivotPosition);
    io.setPivotPosition(state);
    io.updateInputs(inputs);

    Logger.recordOutput("Pivot/Setpoint", targetPosition);
    Logger.recordOutput("Pivot/Output", output);
    Logger.recordOutput("Pivot/CurrentPosition", inputs.pivotPosition);
  }

  /**
   * Manually runs the pivot to the default position with the specified output.
   *
   * @param output Motor output in the range [-1.0, 1.0].
   */
  public void runOpenLoop(double output) {
    io.updateInputs(inputs);
    // Adjust motor output to the pivot manually
    io.setPivotPosition(PivotIOInputs.PivotPositions.Default);
  }
}
