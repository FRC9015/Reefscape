package frc.robot.subsystems.algae.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  @AutoLog
  public static class PivotIOInputs {

    public static enum PivotPosition {
      Default(0.5),
      Down(2);

      private final double pivotPosition;

      PivotPosition(double pivotPosition) {
        this.pivotPosition = pivotPosition;
      }

      public double getPivotPosition() {
        return pivotPosition;
      }
    }

    // Fields Representing the pivot state and inputs
    public PivotPosition pivotState = PivotPosition.Default;
    public double pivotAppliedVolts = 0.0;
    public double pivotCurrentAmps = 0.0;
    public double pivotPosition = 0.0;
    public boolean pivotEncoderConnected = false;
    public boolean pivotAtSetpoint = false;

    public double getDesiredEncoderPosition() {
      return pivotState.getPivotPosition();
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(PivotIOInputs inputs) {}

  /** Sets the pivot position based on the desired state. */
  public default void setPivotPosition(double value) {}

  /** Moves the Pivot Up. */
  public default void pivotUp(double speed) {}

  /** Moves the Pivot Down. */
  public default void pivotDown(double speed) {}

  /** Run slam pivot at amps. */
  default void runCurrent(double amps) {}

  /** Stop slam pivot. */
  default void stop() {}

  /** Enable or disable brake mode on the pivot motor(s). */
  default void setBrakeMode(boolean enable) {}
}
