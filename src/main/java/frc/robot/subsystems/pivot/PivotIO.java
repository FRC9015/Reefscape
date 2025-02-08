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

    // Fields Representing the algae state and inputs
    public double pivotAppliedVolts = 0.0;
    public double pivotCurrentAmps = 0.0;
    public boolean pivotEncoderConnected = false;
    public double pivotPosition = 0.0;
    public boolean pivotAtSetpoint = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(PivotIOInputs inputs) {}

  /** Sets the pivot position based on the desired state. */
  public default void setElevatorPosition(PivotIOInputs.PivotPositions pivotPosition) {}

  /** Run slam pivot at amps */
  default void runCurrent(double amps) {}

  /** Stop slam pivot */
  default void stop() {}

  /** Enable or disable brake mode on the pivot motor(s). */
  default void setBrakeMode(boolean enable) {}
}
