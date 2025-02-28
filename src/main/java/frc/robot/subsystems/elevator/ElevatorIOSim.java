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
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/**
 * Simulation implementation of the ElevatorIO interface using WPILib's ElevatorSim. This simulates
 * the behavior of an elevator subsystem, including physics and voltage control with opposing
 * motors.
 */
public class ElevatorIOSim implements ElevatorIO {
  // Elevator constants
  private static final double ELEVATOR_MIN_HEIGHT = 0.0; // Minimum height (meters)
  private static final double ELEVATOR_MAX_HEIGHT = 3.0; // Maximum height (meters)
  private static final double ELEVATOR_DRUM_RADIUS = 0.025; // Drum radius (meters)
  private static final double ELEVATOR_MASS_KG = 5.0; // Carriage mass (kg)
  private static final double ELEVATOR_GEAR_RATIO = 10.0; // Gear reduction ratio
  private static final double ELEVATOR_STARTING_HEIGHT = 0.0; // Starting position (meters)

  // PID Controller for position control
  private final PIDController elevatorController = new PIDController(10.0, 0.0, 0.0);

  // Elevator simulation model
  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(2),
          ELEVATOR_GEAR_RATIO,
          ELEVATOR_MASS_KG,
          ELEVATOR_DRUM_RADIUS,
          ELEVATOR_MIN_HEIGHT,
          ELEVATOR_MAX_HEIGHT,
          true, // Simulate gravity
          ELEVATOR_STARTING_HEIGHT);

  private boolean positionControl = false;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Run position control if enabled
    if (positionControl) {
      double targetPosition = inputs.getDesiredEncoderPosition();
      appliedVolts = elevatorController.calculate(elevatorSim.getPositionMeters(), targetPosition);
      elevatorSim.setInputVoltage(appliedVolts);
    }

    // Simulate opposing motors
    elevatorSim.update(0.02); // Simulate a 20ms timestep

    // Update inputs with simulated data
    inputs.elevatorEncoderConnected = true; // Simulate that the encoder is always connected
    inputs.elevatorPosition = elevatorSim.getPositionMeters(); // Position in meters
    inputs.elevatorAppliedVolts = appliedVolts; // Voltage applied to the motors
    inputs.elevatorCurrentAmps =
        elevatorSim.getCurrentDrawAmps(); // Approximate current for two motors
  }

  // @Override
  // public void setElevatorPosition(double value) {
  //   positionControl = true;
  //   elevatorController.setSetpoint(state.getEncoderPosition());
  // }
}
