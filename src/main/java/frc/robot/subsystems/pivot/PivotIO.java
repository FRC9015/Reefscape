package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  @AutoLog
  public static class PivotIOInputs {

    public static enum PivotPositions {
      CollectAlgae(0.0),
      PassAlgae(0.5),
      Default(1.0);

      private final double pivotPosition;

      PivotPositions(double pivotPosition) {
        this.pivotPosition = pivotPosition;
      }

      public double getPivotPosition() {
        return pivotPosition;
      }
    }

    // Fields Representing the pivot state and inputs
    public PivotPositions pivotState = PivotPositions.Default;
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
  public default void setPivotPosition(PivotIOInputs.PivotPositions pivotPosition) {}

  /** Run slam pivot at amps */
  default void runCurrent(double amps) {}

  /** Stop slam pivot */
  default void stop() {}

  /** Enable or disable brake mode on the pivot motor(s). */
  default void setBrakeMode(boolean enable) {}
}
