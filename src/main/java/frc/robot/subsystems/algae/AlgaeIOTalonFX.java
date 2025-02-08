package frc.robot.subsystems.algae;

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

public class AlgaeIOTalonFX implements AlgaeIO {
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

  // private final DigitalInput algaeSensor;--> Do we need this?
  public AlgaeIOTalonFX(
      int motorId1, int motorId2, int encoderId1, int encoderId2, String canBusName) {
    motor1 = new TalonFX(motorId1, canBusName);
    motor2 = new TalonFX(motorId2, canBusName);
    encoder1 = new CANcoder(encoderId1, canBusName);
    encoder2 = new CANcoder(encoderId2, canBusName);

    motorFollower = new Follower(motorId1, false);
    motor2.setControl(motorFollower);

    // Configure motors
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motor1.getConfigurator().apply(motorConfig);

    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    followerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motor2.getConfigurator().apply(followerConfig);

    // Configure encoders
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoder1.getConfigurator().apply(encoderConfig);
    encoder2.getConfigurator().apply(encoderConfig);

    rpmSignal = encoder1.getVelocity();
    appliedVoltsSignal = motor1.getMotorVoltage();
    currentSignal = motor1.getStatorCurrent();
  }

  @Override
  public void updateInputs(AlgaeIOInputs inputs) {
    // Refresh signals
    var encoderStatus = BaseStatusSignal.refreshAll(rpmSignal);

    // Update inputs
    inputs.algaeEncoderConnected = encoderConnectedDebounce.calculate(encoderStatus.isOK());
    inputs.algaeRPM = rpmSignal.getValueAsDouble();
    inputs.algaeAppliedVolts = appliedVoltsSignal.getValueAsDouble();
    inputs.algaeCurrentAmps = currentSignal.getValueAsDouble();
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
