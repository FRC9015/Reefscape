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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs.ElevatorState;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private PIDController pidController;
  private final Alert encoderDisconnectedAlert;

  // Elevator PID constants - Initial values
  private double kP = 1.0;
  private double kI = 0.0;
  private double kD = 0.0;
  private static final double kToleranceMeters = 0.01; // Acceptable position error in meters

  private ShuffleboardTab elevatorTab; // Shuffleboard Tab

  public Elevator(ElevatorIO io) {
    this.io = io;
    this.pidController = new PIDController(kP, kI, kD);
    this.pidController.setTolerance(kToleranceMeters);
    this.setDefaultCommand(executePreset(ElevatorState.Default));

    encoderDisconnectedAlert = new Alert("Disconnected elevator encoder.", AlertType.kError);

    // Create a Shuffleboard tab for the elevator
    elevatorTab = Shuffleboard.getTab("Elevator");

    // Add PID controller values to shuffleboard
    elevatorTab.add("Elevator P Gain", kP);
    elevatorTab.add("Elevator I Gain", kI);
    elevatorTab.add("Elevator D Gain", kD);
  }

  /** Periodic method to update inputs, PID calculations, and alerts. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // Update alert
    encoderDisconnectedAlert.set(!inputs.elevatorEncoderConnected);

    // Update PID values from Shuffleboard
    updatePIDValues();
  }

  private void updatePIDValues() {
    double newP = elevatorTab.add("Elevator P Gain", kP).getEntry().getDouble(kP);
    double newI = elevatorTab.add("Elevator I Gain", kI).getEntry().getDouble(kI);
    double newD = elevatorTab.add("Elevator D Gain", kD).getEntry().getDouble(kD);

    if (newP != kP || newI != kI || newD != kD) {
        kP = newP;
        kI = newI;
        kD = newD;
        pidController.setPID(kP, kI, kD);
    }
  }

  /**
   * Sets the elevator to the specified preset state using PID control.
   *
   * @param state The desired preset state.
   */
  public void setPreset(ElevatorIOInputs.ElevatorState state) {
    double targetPosition = state.getEncoderPosition();
    pidController.setSetpoint(targetPosition);
    io.setElevatorPosition(pidController.calculate(inputs.elevatorPosition));
    io.updateInputs(inputs);
    Logger.recordOutput("Elevator/Setpoint", targetPosition);
  }

  public Command executePreset(ElevatorIOInputs.ElevatorState state) {
    Logger.recordOutput("Elevator/State", state);
    Logger.recordOutput("Elevator/CurrentPosition", inputs.elevatorPosition);
    return run(() -> this.setPreset(state));
  }
}
