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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * The IntakeIOTalonFX class implements the IntakeIO interface and controls the intake subsystem
 * using TalonFX motors and CANcoder encoders.
 */
public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX motor1;
  private final TalonFX motor2;
  private final CANcoder encoder1;
  private final CANcoder encoder2;
  private final Follower motorFollower;
  private final NeutralOut neutralOut = new NeutralOut();
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

  private final StatusSignal<AngularVelocity> rpmSignal;
  private final StatusSignal<Voltage> appliedVoltsSignal;
  private final StatusSignal<Current> currentSignal;

  private final Debouncer encoderConnectedDebounce = new Debouncer(0.5);
  private final DigitalInput coralSensor;

  /**
   * Constructs an IntakeIOTalonFX object.
   *
   * @param motorId1 The ID of the first motor.
   * @param motorId2 The ID of the second motor.
   * @param encoderId1 The ID of the first encoder.
   * @param encoderId2 The ID of the second encoder.
   * @param coralSensorChannel The channel of the coral sensor.
   * @param canBusName The name of the CAN bus.
   */
  public IntakeIOTalonFX(
      int motorId1,
      int motorId2,
      int encoderId1,
      int encoderId2,
      int coralSensorChannel,
      String canBusName) {
    motor1 = new TalonFX(motorId1, canBusName);
    motor2 = new TalonFX(motorId2, canBusName);
    encoder1 = new CANcoder(encoderId1, canBusName);
    encoder2 = new CANcoder(encoderId2, canBusName);
    coralSensor = new DigitalInput(coralSensorChannel);

    motorFollower = new Follower(motorId1, false);
    motor2.setControl(motorFollower);

    // Configure motors
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motor1.getConfigurator().apply(motorConfig);

    TalonFXConfiguration motorConfig2 = new TalonFXConfiguration();
    motorConfig2.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motor2.getConfigurator().apply(motorConfig2);

    // Configure encoders
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoder1.getConfigurator().apply(encoderConfig);

    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoder2.getConfigurator().apply(encoderConfig);

    // Signals
    rpmSignal = encoder1.getVelocity();
    appliedVoltsSignal = motor1.getMotorVoltage();
    currentSignal = motor1.getStatorCurrent();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Refresh signals
    var encoderStatus = BaseStatusSignal.refreshAll(rpmSignal);

    // Update inputs
    inputs.intakeEncoderConnected = encoderConnectedDebounce.calculate(encoderStatus.isOK());
    inputs.intakeRPM = rpmSignal.getValueAsDouble();
    inputs.intakeAppliedVolts = appliedVoltsSignal.getValueAsDouble();
    inputs.intakeCurrentAmps = currentSignal.getValueAsDouble();
    inputs.coralDetected =
        !coralSensor.get(); // Coral detected if the sensor is triggered (active low)
  }

  @Override
  public void stop() {
    motor1.setControl(neutralOut);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    motor1.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    motor2.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setRPM(double rpm) {
    motor1.setControl(velocityRequest.withVelocity(rpm));
  }
}
