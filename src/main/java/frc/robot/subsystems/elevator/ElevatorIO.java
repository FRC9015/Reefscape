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

/** Interface representing the elevator subsystem's input/output operations. */
public interface ElevatorIO {
  
  /**
   * Class containing all loggable inputs for the elevator.
   */
  @AutoLog
  public static class ElevatorIOInputs {
    
    /**
     * Enum representing different elevator preset positions.
     */
    public enum ElevatorState {
      Default(0.0),
      CoralL2(1.0),
      DealgifyL2(1.5),
      CoralL3(2.0),
      DealgifyL3(2.5),
      CoralL4(3.0);

      /** The encoder position associated with this state. */
      private final double encoderPosition;

      /**
       * Constructs an ElevatorState with an associated encoder position.
       *
       * @param encoderPosition The encoder position in meters.
       */
      ElevatorState(double encoderPosition) {
        this.encoderPosition = encoderPosition;
      }

      /**
       * Gets the encoder position associated with this state.
       *
       * @return The encoder position in meters.
       */
      public double getEncoderPosition() {
        return encoderPosition;
      }
    }

    /** The current elevator state. */
    public ElevatorState elevatorState = ElevatorState.Default;
    
    /** The voltage applied to the elevator motor. */
    public double elevatorAppliedVolts = 0.0;
    
    /** The current drawn by the elevator motor in amps. */
    public double elevatorCurrentAmps = 0.0;
    
    /** Whether the elevator encoder is connected. */
    public boolean elevatorEncoderConnected = false;
    
    /** The current position of the elevator in meters. */
    public double elevatorPosition = 0.0;
    
    /** Whether the elevator has reached its setpoint. */
    public boolean elevatorAtSetpoint = false;

    /**
     * Gets the encoder position associated with the current elevator state.
     *
     * @return The desired encoder position in meters.
     */
    public double getDesiredEncoderPosition() {
      return elevatorState.getEncoderPosition();
    }
  }

  /**
   * Updates the set of loggable inputs for the elevator.
   *
   * @param inputs The {@link ElevatorIOInputs} instance to update.
   */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * Sets the elevator to a specified preset position.
   *
   * @param state The desired {@link ElevatorIOInputs.ElevatorState}.
   */
  public default void setElevatorPosition(ElevatorIOInputs.ElevatorState state) {}

  /**
   * Runs the elevator using current control (slam mode).
   *
   * @param amps The desired current in amps.
   */
  default void runCurrent(double amps) {}

  /** Stops the elevator. */
  default void stop() {}

  /**
   * Enables or disables brake mode on the elevator motor.
   *
   * @param enable {@code true} to enable brake mode, {@code false} to disable.
   */
  default void setBrakeMode(boolean enable) {}
}
