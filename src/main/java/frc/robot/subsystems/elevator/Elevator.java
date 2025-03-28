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

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private double kP = 5;
  private double kI = 0.0;
  private double kD = 0.0;
  private double kS = 0;
  private double kG = 1;
  private double kV = 0;
  private double kA = 0.0;
  private double offset = 0.0;
  double lastPosition = 0;
  int stalledCount = 0;

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
   * the.
   *
   * @param state The desired preset state.
   */
  public void setPreset(ElevatorIOInputs.ElevatorState state) {

    double targetPosition = state.getEncoderPosition();
    // if (Math.abs(lastPosition - inputs.elevatorPosition) <= 0.03) {
    //   stalledCount += 1;
    // }

    // if (stalledCount >= 20) {
    //   targetPosition = inputs.elevatorPosition;
    //   stalledCount = 0;
    // }

    pidController.setSetpoint(targetPosition);

    double output =
        pidController.calculate(inputs.elevatorPosition)
            + feedforward.calculate(inputs.elevatorPosition);
    inputs.setpoint = targetPosition;
    if (Math.abs(targetPosition - inputs.elevatorPosition + offset) <= 0.03) {
      inputs.elevatorAtSetpoint = true;
    }
    io.setElevatorPosition(output);
    io.updateInputs(inputs);
    lastPosition = inputs.elevatorPosition;

    Logger.recordOutput("Elevator/Setpoint", targetPosition);
    Logger.recordOutput("Elevator/PID Output", output);
  }

  /**
   * Executes a command to set the elevator to a preset state.
   *
   * @param state The desired preset state.
   * @return A command that sets the elevator to the preset state.
   */
  public Command executePreset(ElevatorIOInputs.ElevatorState state) {
    Logger.recordOutput("Elevator/State", state);
    Logger.recordOutput("Elevator/CurrentPosition", inputs.elevatorPosition + offset);
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

  /**
   * Gets the current state of the elevator.
   *
   * @return The current state of the elevator.
   */
  public ElevatorState getElevatorState() {
    return inputs.elevatorState;
  }
}
