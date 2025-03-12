// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {

    // Fields representing the intake state and inputs
    public double intakeAppliedVolts = 0.0;
    public double intakeCurrentAmps = 0.0;
    public boolean intakeEncoderConnected = false;
    public double intakeRPM = 0.0;
    public boolean coralIn = false;
    public boolean coralSet = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Stop intake`from running */
  default void stop() {}

  /** Enable or disable brake mode on the intake motor. */
  default void setBrakeMode(boolean enable) {}

  default void setRPM(double rpm) {}
}
