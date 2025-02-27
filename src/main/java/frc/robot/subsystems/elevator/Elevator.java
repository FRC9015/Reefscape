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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs.ElevatorState;
import org.littletonrobotics.junction.Logger;

/**
 * The Elevator subsystem controls the elevator mechanism of the robot. It uses a PID controller and
 * feedforward to achieve precise positioning.
 */
public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private PIDController pidController;
  private final Alert encoderDisconnectedAlert;
  private ElevatorFeedforward feedforward;

  // Elevator PID constants - Initial values
  private double kP = 4.5;
  private double kI = 0.0;
  private double kD = 0.05;
  private double kS = 0;
  private double kG = 1;
  private double kV = 0;
  private double kA = 0.0;
  private double offset = 0;

  private static final double kToleranceMeters = 0.01; // Acceptable position error in meters

  /**
   * Constructs an Elevator subsystem.
   *
   * @param io The ElevatorIO instance used for input/output operations.
   */
  public Elevator(ElevatorIO io) {
    this.io = io;
    this.pidController = new PIDController(kP, kI, kD);
    this.feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
    this.pidController.setTolerance(kToleranceMeters);
    this.setDefaultCommand(executePreset(ElevatorState.Default));

    encoderDisconnectedAlert = new Alert("Disconnected elevator encoder.", AlertType.kError);
  }

  /** Periodic method to update inputs, PID calculations, and alerts. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // Update alert
    encoderDisconnectedAlert.set(!inputs.elevatorEncoderConnected);
  }

  /**
   * Sets the elevator to a preset state. added fixed logic for L3 (encoder position 3.82) and L4
   * (7.375) When the elevator is between these two positions, the output is scaled from
   * Constants.SLOW_MODE_CONSTANT (at L3) to Constants.ANTI_CAPSIZE_CONSTANT (at L4), but stays at
   * Constants.SLOW_MODE_CONSTANT for L3 and moving to L4.
   *
   * @param state The desired preset state.
   */
  public void setPreset(ElevatorIOInputs.ElevatorState state) {
    double targetPosition = state.getEncoderPosition();
    pidController.setSetpoint(targetPosition);
    double output =
        pidController.calculate(inputs.elevatorPosition)
            + feedforward.calculate(inputs.elevatorPosition);

    inputs.setpoint = targetPosition;
    if (Math.abs(targetPosition - inputs.elevatorPosition) <= 0.03) {
      inputs.elevatorAtSetpoint = true;
      // Interpolated slow mode scaling for L3 to L4
      if (state == ElevatorIOInputs.ElevatorState.CoralL3
          || state == ElevatorIOInputs.ElevatorState.CoralL4) {
        double l3Pos =
            ElevatorIOInputs.ElevatorState.CoralL3.getEncoderPosition(); // Expected: 3.82
        double l4Pos =
            ElevatorIOInputs.ElevatorState.CoralL4.getEncoderPosition(); // Expected: 7.375

        // Keep only at current position between L3 and L4
        double currentPos = MathUtil.clamp(inputs.elevatorPosition, l3Pos, l4Pos);
        // Compute the interpolation fraction (0 at L3, 1 at L4)
        double fraction = (currentPos - l3Pos) / (l4Pos - l3Pos);
        // Linearly interpolate scaling factor at L3 use SLOW_MODE_CONSTANT gradually increasing at
        // L4
        // to ANTI_CAPSIZE_CONSTANT
        double scalingFactor =
            (1 - fraction) * Constants.SLOW_MODE_CONSTANT
                + fraction * Constants.ANTI_CAPSIZE_CONSTANT;
        output *= scalingFactor;
      }
    }
    // Interpolated slow mode scaling for L3 to L4

    io.setElevatorPosition(output);
    io.updateInputs(inputs);
    Logger.recordOutput("Elevator/Setpoint", targetPosition);
  }

  /**
   * Executes a command to set the elevator to a preset state.
   *
   * @param state The desired preset state.
   * @return A command that sets the elevator to the preset state.
   */
  public Command executePreset(ElevatorIOInputs.ElevatorState state) {
    Logger.recordOutput("Elevator/State", state);
    Logger.recordOutput("Elevator/CurrentPosition", inputs.elevatorPosition);
    return run(() -> this.setPreset(state));
  }

  /**
   * Checks if the elevator is at the set position.
   *
   * @return True if the elevator is at the set position, false otherwise.
   */
  public Boolean elevatorAtPosition() {
    return inputs.elevatorAtSetpoint;
  }

  /**
   * Zeros the elevator position.
   *
   * @return A command that zeros the elevator.
   */
  public Command zeroTheElevator() {
    return run(() -> io.zeroElevator());
  }

  public Boolean getElevatorLimitSwitch() {
    return inputs.zeroSwitchTriggered;
  }
}
