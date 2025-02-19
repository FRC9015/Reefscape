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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final PIDController pidController;
  private final Alert encoderDisconnectedAlert;

  // Elevator PID constants
  private static final double kP = 10.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kToleranceMeters = 0.01; // Acceptable position error in meters

  public Elevator(ElevatorIO io) {
    this.io = io;
    this.pidController = new PIDController(kP, kI, kD);
    this.pidController.setTolerance(kToleranceMeters);

    encoderDisconnectedAlert = new Alert("Disconnected elevator encoder.", AlertType.kError);
  }

  /** Periodic method to update inputs, PID calculations, and alerts. */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // Update alert
    encoderDisconnectedAlert.set(!inputs.elevatorEncoderConnected);
  }

  /**
   * Sets the elevator to the specified preset state using PID control.
   *
   * @param state The desired preset state.
   */
  public void setPreset(ElevatorIOInputs.ElevatorState state) {
    double targetPosition = state.getEncoderPosition();
    pidController.setSetpoint(targetPosition);
    double output = pidController.calculate(inputs.elevatorPosition);
    io.setElevatorPosition(state);
    io.updateInputs(inputs);
    Logger.recordOutput("Elevator/Setpoint", targetPosition);
  }

  /**
   * Manually runs the elevator with the specified output.
   *
   * @param output Motor output in the range [-1.0, 1.0].
   */
  public void runOpenLoop(double output) {
    io.updateInputs(inputs);
    Logger.recordOutput("Elevator/Output", output);
    Logger.recordOutput("Elevator/CurrentPosition", inputs.elevatorPosition);
    // Adjust motor output to the elevator manually
    io.setElevatorPosition(ElevatorIOInputs.ElevatorState.Default);
  }

  public Command executePreset(ElevatorIOInputs.ElevatorState state) {
    return run(() -> this.setPreset(state));
  }
}
