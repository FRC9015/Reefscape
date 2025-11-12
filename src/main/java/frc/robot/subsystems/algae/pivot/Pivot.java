package frc.robot.subsystems.algae.pivot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algae.pivot.PivotIO.PivotIOInputs;
import frc.robot.subsystems.algae.pivot.PivotIO.PivotIOInputs.PivotPosition;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotIO io;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private final Alert encoderDisconnectedAlert;

  public Pivot(PivotIO io) {
    this.io = io;
    this.setDefaultCommand(executePreset(PivotPosition.Default));
    encoderDisconnectedAlert = new Alert("Pivot encoder disconnected!", AlertType.kError);
  }

  /** Periodically updates the algae's state and logs inputs. */
  @Override
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
    io.setPivotPosition(targetPosition);
    io.updateInputs(inputs);

    Logger.recordOutput("Pivot/Setpoint", targetPosition);
    Logger.recordOutput("Pivot/CurrentPosition", inputs.pivotPosition);
  }

  public Command pivotUp(double speed) {
    Logger.recordOutput("Pivot/speed", speed);
    Logger.recordOutput("Pivot/CurrentPosition", inputs.pivotPosition);
    return startEnd(() -> io.pivotUp(speed), () -> io.pivotUp(0));
  }

  public Command pivotDown(double speed) {
    Logger.recordOutput("Pivot/speed", speed);
    Logger.recordOutput("Pivot/CurrentPosition", inputs.pivotPosition);
    return startEnd(() -> io.pivotDown(-speed), () -> io.pivotDown(0));
  }

  public Command executePreset(PivotIOInputs.PivotPosition state) {
    Logger.recordOutput("Pivot/State", state);
    Logger.recordOutput("Pivot/CurrentPosition", inputs.pivotPosition);
    return run(() -> this.setPreset(state));
  }

  public double getPivotPosition() {
    return inputs.pivotPosition;
  }
}
