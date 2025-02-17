package frc.robot.subsystems.algae;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
  private final TalonFX motor;
  private final CANcoder encoder;
  private final NeutralOut neutralOut = new NeutralOut();
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

  private final StatusSignal<AngularVelocity> rpmSignal;
  private final StatusSignal<Voltage> appliedVoltsSignal;
  private final StatusSignal<Current> currentSignal;

  private final Debouncer encoderConnectedDebounce = new Debouncer(0.5);

  // private final DigitalInput algaeSensor;--> Do we need this?
  public AlgaeIOTalonFX(int motorId1, int encoderId1, String canBusName) {
    motor = new TalonFX(motorId1, canBusName);
    encoder = new CANcoder(encoderId1, canBusName);

    // Configure motors
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motor.getConfigurator().apply(motorConfig);

    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    followerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Configure encoders
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoder.getConfigurator().apply(encoderConfig);

    rpmSignal = encoder.getVelocity();
    appliedVoltsSignal = motor.getMotorVoltage();
    currentSignal = motor.getStatorCurrent();
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
    motor.setControl(neutralOut);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    motor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setRPM(double rpm) {
    motor.setControl(velocityRequest.withVelocity(rpm));
  }
}
