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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ElevatorConstants;

/** the. */
public class ElevatorIOTalonFX implements ElevatorIO {

  private final DigitalInput zeroSwitch;
  private final TalonFX elevatorMotor;
  private final TalonFX elevatorMotor1;
  private final CANcoder elevatorEncoder;

  private final StatusSignal<Angle> encoderPositionSignal;
  private final StatusSignal<Voltage> motorAppliedVoltsSignal;
  private final StatusSignal<Current> motorCurrentSignal;
  private final StatusSignal<Angle> motorPosition;

  private final Debouncer encoderConnectedDebounce = new Debouncer(0.5);
  private final NeutralOut neutralOut = new NeutralOut();
  private final VoltageOut voltageOut = new VoltageOut(0.0);
  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.5);

  SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs =
      new SoftwareLimitSwitchConfigs()
          .withForwardSoftLimitEnable(true)
          .withForwardSoftLimitThreshold(ElevatorConstants.maxHeight)
          .withReverseSoftLimitEnable(true)
          .withReverseSoftLimitThreshold(ElevatorConstants.minHeight);

  /**
   * Constructs an ElevatorIOTalonFX.
   *
   * @param motorID The ID of the first motor.
   * @param followMotorID The ID of the second motor.
   * @param encoderId The ID of the encoder.
   * @param limitSwitchPort The port number of the limit switch.
   */
  public ElevatorIOTalonFX(int motorID, int followMotorID, int encoderId, int limitSwitchPort) {
    zeroSwitch = new DigitalInput(limitSwitchPort);
    elevatorMotor = new TalonFX(motorID);
    elevatorEncoder = new CANcoder(encoderId);
    elevatorMotor1 = new TalonFX(followMotorID);
    elevatorMotor1.setControl(new Follower(motorID, true));

    // Configure the motor
    TalonFXConfiguration motorConfig =
        new TalonFXConfiguration()
            .withSoftwareLimitSwitch(softwareLimitSwitchConfigs)
            .withMotionMagic(ElevatorConstants.MOTION_MAGIC_CONFIGS)
            .withSlot0(ElevatorConstants.SLOT0_CONFIGS)
            .withFeedback(ElevatorConstants.FEEDBACK_CONFIGS)
            .withClosedLoopRamps(new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(0.1));
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    elevatorMotor.getConfigurator().apply(motorConfig);
    elevatorMotor1.getConfigurator().apply(motorConfig);
    // Configure the encoder
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.MagnetOffset = ElevatorConstants.ELEVATOR_MAGNET_OFFSET;
    elevatorEncoder.getConfigurator().apply(encoderConfig);

    // Signals
    encoderPositionSignal = elevatorEncoder.getPosition();
    motorAppliedVoltsSignal = elevatorMotor.getMotorVoltage();
    motorCurrentSignal = elevatorMotor.getStatorCurrent();
    motorPosition = elevatorMotor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, encoderPositionSignal, motorAppliedVoltsSignal, motorCurrentSignal, motorPosition);
    elevatorEncoder.optimizeBusUtilization();
    elevatorMotor.optimizeBusUtilization();
    elevatorMotor1.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Refresh signals
    StatusCode encoderStatus =
        BaseStatusSignal.refreshAll(
            encoderPositionSignal, motorAppliedVoltsSignal, motorCurrentSignal, motorPosition);

    // Update elevator inputs
    inputs.elevatorEncoderConnected = encoderConnectedDebounce.calculate(encoderStatus.isOK());
    inputs.elevatorPosition = encoderPositionSignal.getValueAsDouble();
    inputs.elevatorAppliedVolts = motorAppliedVoltsSignal.getValueAsDouble();
    inputs.elevatorCurrentAmps = motorCurrentSignal.getValueAsDouble();
    inputs.elevatorMotorPosition = motorPosition.getValueAsDouble();
  }

  @Override
  public void setElevatorPosition(double value) {
    // elevatorMotor.setControl(voltageOut.withOutput(MathUtil.clamp(value, -12, 12)));
    if (value > ElevatorConstants.maxHeight) {
      value = ElevatorConstants.maxHeight;
    }
    if (value < ElevatorConstants.minHeight) {
      value = ElevatorConstants.minHeight;
    }
    elevatorMotor.setControl(motionMagicVoltage.withPosition(value));
  }

  @Override
  public void stop() {
    elevatorMotor.setControl(neutralOut);
  }

  @Override
  public void setBrakeMode() {
    elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    elevatorMotor1.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void setCoastMode() {
    elevatorMotor.setNeutralMode(NeutralModeValue.Coast);
    elevatorMotor1.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void getZeroSwitch(ElevatorIOInputs inputs) {
    inputs.zeroSwitchTriggered = zeroSwitch.get();
  }

  @Override
  public void zeroElevator() {
    elevatorEncoder.setPosition(0); // Zero the CANcoder encoder
  }
}
