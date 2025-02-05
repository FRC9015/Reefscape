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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Simulates the intake subsystem using DC motor simulations. */
public class IntakeIOSim implements IntakeIO {
  private static final double MAX_RPM = 3000.0;
  private static final double GEAR_REDUCTION = 1.0; // Adjust if gearing is used
  private static final double MOMENT_OF_INERTIA = 0.001; // kg * m^2, adjust as needed

  private final DCMotorSim motorSim1;
  private final DCMotorSim motorSim2;
  private double appliedVolts = 0.0;
  private boolean brakeMode = false;

  /** Constructs an IntakeIOSim instance. Creates two DCMotorSim instances for Kraken X60 motors. */
  public IntakeIOSim() {
    // Create two DCMotorSim instances for Kraken X60 motors
    motorSim1 =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), MOMENT_OF_INERTIA, GEAR_REDUCTION),
            DCMotor.getKrakenX60(1));
    motorSim2 =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), MOMENT_OF_INERTIA, GEAR_REDUCTION),
            DCMotor.getKrakenX60(1));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Apply brake mode behavior if enabled and no voltage is applied
    if (brakeMode && appliedVolts == 0.0) {
      motorSim1.setInputVoltage(0.0);
      motorSim1.setAngularVelocity(0.0); // Simulate stopping the motor
      motorSim2.setInputVoltage(0.0);
      motorSim2.setAngularVelocity(0.0);
    } else {
      motorSim1.setInputVoltage(appliedVolts);
      motorSim2.setInputVoltage(-appliedVolts);
    }

    // Update motor simulations
    motorSim1.update(0.02); // 20ms update rate
    motorSim2.update(0.02);

    // Calculate average RPM (motors running in opposite directions)
    double avgRPM = (motorSim1.getAngularVelocityRPM() - motorSim2.getAngularVelocityRPM()) / 2.0;

    // Update inputs
    inputs.intakeAppliedVolts = appliedVolts;
    inputs.intakeCurrentAmps =
        (motorSim1.getCurrentDrawAmps() + motorSim2.getCurrentDrawAmps()) / 2.0;
    inputs.intakeEncoderConnected = true;
    inputs.intakeRPM = avgRPM;
    inputs.coralDetected = false; // Simulate coral detection if needed
  }

  @Override
  public void stop() {
    setRPM(0);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    this.brakeMode = enable;
  }

  @Override
  public void setRPM(double rpm) {
    // Limit RPM to MAX_RPM
    double limitedRPM = MathUtil.clamp(rpm, -MAX_RPM, MAX_RPM);
    double limitedRadPerSec = limitedRPM / 60 * 2 * Math.PI;

    // Apply voltages to motor simulations
    motorSim1.setAngularVelocity(limitedRadPerSec);
    motorSim2.setAngularVelocity(-limitedRadPerSec);

    // Adjust applied voltage based on RPM
    appliedVolts = limitedRPM / MAX_RPM * 12.0; // Approximation for voltage based on RPM
  }
}
