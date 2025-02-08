package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.pivot.PivotIO.PivotIOInputs;

public interface AlgaeIO {
  @AutoLog
  public static class AlgaeIOInputs {
    // Fields Representing the algae state and inputs
    public double algaeAppliedVolts = 0.0;
    public double algaeCurrentAmps = 0.0;
    public boolean algaeEncoderConnected = false;
    public double algaePosition = 0.0;
    public double algaeRPM = 0.0;
    public boolean algaeAtSetpoint = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(AlgaeIOInputs inputs) {}

  /** Run slam algae at amps */
  default void runCurrent(double amps) {}

  /** Stop slam algae */
  default void stop() {}

  /** Enable or disable brake mode on the algae motor. */
  default void setBrakeMode(boolean enable) {}

  default void setRPM(double rpm) {}
}
