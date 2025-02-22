package frc.robot.subsystems.algae.pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algae.pivot.PivotIO.PivotIOInputs;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private final Alert encoderDisconnectedAlert;
  private final PIDController pidController;

  // Pivot PID constants
  private static final double kP = 1.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kToleranceMeters = 0.01; // Acceptable position error in meters

  public Pivot(PivotIO io) {
    this.io = io;
    this.pidController = new PIDController(kP, kI, kD);
    this.pidController.setTolerance(kToleranceMeters);
    // this.setDefaultCommand(executePreset(PivotPosition.Default));
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
  public void setPreset(PivotIOInputs.PivotPosition state) {
    double targetPosition = state.getPivotPosition();
    double output = pidController.calculate(inputs.pivotPosition);
    io.setPivotPosition(output);
    pidController.setSetpoint(output);
    io.updateInputs(inputs);

    Logger.recordOutput("Pivot/Setpoint", targetPosition);
    Logger.recordOutput("Pivot/Output", output);
    Logger.recordOutput("Pivot/CurrentPosition", inputs.pivotPosition);
  }

  public Command pivotUp(double speed) {
    Logger.recordOutput("Pivor/speed", speed);
    Logger.recordOutput("Pivot/CurrentPosition", inputs.pivotPosition);
    return run(() -> io.pivotUp(speed));
  }

  public Command pivotDown(double speed) {
    Logger.recordOutput("Pivor/speed", speed);
    Logger.recordOutput("Pivot/CurrentPosition", inputs.pivotPosition);
    return run(() -> io.pivotDown(speed));
  }

  public Command executePreset(PivotIOInputs.PivotPosition state) {
    Logger.recordOutput("Pivot/State", state);
    Logger.recordOutput("Pivot/CurrentPosition", inputs.pivotPosition);
    return run(() -> this.setPreset(state));
  }
}
