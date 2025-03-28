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

package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** the. */
public class EndEffectorIOTalonFX implements EndEffectorIO {

  private final TalonFX motor1;
  // private final CANrange sideRange1;
  // private final CANrange middleRange;
  // private final CANrange sideRange2;

  private final StatusSignal<AngularVelocity> rpmSignal;
  private final StatusSignal<Voltage> appliedVoltsSignal;
  private final StatusSignal<Current> currentSignal;

  private final Debouncer encoderConnectedDebounce = new Debouncer(0.5);

  /**
   * Constructs an EndEffectorIOTalonFX.
   *
   * @param motorId1 The ID of the motor.
   */
  public EndEffectorIOTalonFX(int motorId1) { // , int canRangeID1, int canRangeID2, int canRangeID3
    motor1 = new TalonFX(motorId1, "CANivore");
    // sideRange1 = new CANrange(canRangeID1);
    // middleRange = new CANrange(canRangeID2);
    // sideRange2 = new CANrange(canRangeID3);

    // Configure motor
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Configure the integrated encoder (default settings should work)
    motor1.getConfigurator().apply(motorConfig);

    CANrangeConfiguration rangeConfig = new CANrangeConfiguration();
    rangeConfig.FovParams.FOVRangeX = 15;

    // sideRange1.getConfigurator().apply(rangeConfig);
    // sideRange2.getConfigurator().apply(rangeConfig);
    // middleRange.getConfigurator().apply(rangeConfig);

    // Use the built-in relative encoder of the TalonFX
    rpmSignal = motor1.getVelocity();
    appliedVoltsSignal = motor1.getMotorVoltage();
    currentSignal = motor1.getStatorCurrent();
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    // Refresh signals
    StatusCode encoderStatus = BaseStatusSignal.refreshAll(rpmSignal);

    // Update inputs
    inputs.endEffectorEncoderConnected = encoderConnectedDebounce.calculate(encoderStatus.isOK());
    inputs.endEffectorRPM = rpmSignal.getValueAsDouble();
    inputs.endEffectorAppliedVolts = appliedVoltsSignal.getValueAsDouble();
    inputs.endEffectorCurrentAmps = currentSignal.getValueAsDouble();
    // inputs.canRange1 = sideRange1.getIsDetected().getValue();
    // inputs.canRange2 = sideRange2.getIsDetected().getValue();
    // inputs.canRange3 = middleRange.getIsDetected().getValue();

  }

  @Override
  public void stop() {
    motor1.stopMotor();
  }

  @Override
  public void setBrakeMode(boolean enable) {
    motor1.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setRPM(double voltage) {
    motor1.setVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
  }
}
