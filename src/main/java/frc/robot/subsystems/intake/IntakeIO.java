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
    public boolean coralFound = false;
    public boolean coralSet = false;
    public double middleSTDdevs = 0.0;
    public double middleDistance = 0.0;
    public boolean middleIsDetected = false;
    public double side1STDdevs = 0.0;
    public double side1Distance = 0.0;
    public boolean side1IsDetected = false;
    public double side2STDdevs = 0.0;
    public double side2Distance = 0.0;
    public boolean side2IsDetected = false;
  }

  // ...existing code...
  public static enum CanRangeStates {
    AllClear(false, false, false),
    MiddleOnly(true, false, false),
    Side1Only(false, true, false),
    Side2Only(false, false, true),
    MiddleAndSide1(true, true, false),
    MiddleAndSide2(true, false, true),
    Side1AndSide2(false, true, true),
    AllBlocked(true, true, true),
    Default(false, false, false); // used when inputs are unknown/null

    public final boolean middleDetected;
    public final boolean side1Detected;
    public final boolean side2Detected;

    CanRangeStates(boolean middleDetected, boolean side1Detected, boolean side2Detected) {
      this.middleDetected = middleDetected;
      this.side1Detected = side1Detected;
      this.side2Detected = side2Detected;
    }

    /** Determine state from possibly-null Boolean flags. null -> Default */
    public static CanRangeStates fromBooleans(Boolean middle, Boolean side1, Boolean side2) {
      if (middle == null || side1 == null || side2 == null) {
        return Default;
      }
      for (CanRangeStates s : values()) {
        if (s == Default) continue;
        if (s.middleDetected == middle && s.side1Detected == side1 && s.side2Detected == side2) {
          return s;
        }
      }
      return Default;
    }

    /** Convenience: determine state from IntakeIOInputs */
    public static CanRangeStates fromInputs(IntakeIOInputs inputs) {
      if (inputs == null) return Default;
      return fromBooleans(inputs.middleIsDetected, inputs.side1IsDetected, inputs.side2IsDetected);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Stop intake`from running */
  default void stop() {}

  /** Enable or disable brake mode on the intake motor. */
  default void setBrakeMode(boolean enable) {}

  default void setRPM(double rpm) {}
}
