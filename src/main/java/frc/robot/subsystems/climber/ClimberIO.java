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

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/** the. */
public interface ClimberIO {

  /** the. */
  @AutoLog
  public static class ClimberIOInputs {

    // Fields representing the intake state and inputs
    public double climberAppliedVolts = 0.0;
    public double climberCurrentAmps = 0.0;
    public double climberRPM = 0.0;
    public double climberPosition = 0.0;
    public double servoPosition = 0.0;
    public double servoAngle = 0.0;
    public double servoSpeed = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /** Stop intake`from running. */
  default void stop() {}

  /** Enable or disable brake mode on the intake motor. */
  default void setBrakeMode(boolean enable) {}

  default void setTopRPM(double rpm) {}

  default void setClimbRPM(double rpm) {}

  default void servoOpen() {}
  ;

  default void servoClose() {}
  ;
}
