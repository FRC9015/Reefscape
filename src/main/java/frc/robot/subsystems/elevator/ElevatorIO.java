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

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    // Define ElevatorState enum with encoder positions
    public enum ElevatorState {
      Default(0.0),
      CoralL2(1.0),
      DealgifyL2(1.5),
      CoralL3(2.0),
      DealgifyL3(2.5),
      CoralL4(3.0);

      // Field to store the encoder position
      private final double encoderPosition;

      // Constructor to assign the encoder position
      ElevatorState(double encoderPosition) {
        this.encoderPosition = encoderPosition;
      }

      // Getter for encoder position
      public double getEncoderPosition() {
        return encoderPosition;
      }
    }

    // Fields representing the elevator state and inputs
    public ElevatorState elevatorState = ElevatorState.Default;
    public double elevatorAppliedVolts = 0.0;
    public double elevatorCurrentAmps = 0.0;
    public boolean elevatorEncoderConnected = false;
    public double elevatorPosition = 0.0;
    public boolean elevatorAtSetpoint = false;

    // Utility method to get the desired encoder position for the current state
    public double getDesiredEncoderPosition() {
      return elevatorState.getEncoderPosition();
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Sets the elevator position based on the desired state. */
  public default void setElevatorPosition(ElevatorIOInputs.ElevatorState state) {}

  /** Run slam elevator at amps */
  default void runCurrent(double amps) {}

  /** Stop slam elevator */
  default void stop() {}

  /** Enable or disable brake mode on the elevator motor. */
  default void setBrakeMode(boolean enable) {}
}