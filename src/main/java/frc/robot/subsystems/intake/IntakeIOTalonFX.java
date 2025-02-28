package frc.robot.subsystems.intake;

// import com.ctre.phoenix6.BaseStatusSignal;
// import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.NeutralOut;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import edu.wpi.first.math.filter.Debouncer;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.units.measure.Current;
// import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOTalonFX implements IntakeIO {

  // public final TalonFX motor;
  // private final NeutralOut neutralOut = new NeutralOut();
  // private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

  // private final StatusSignal<AngularVelocity> rpmSignal;
  // private final StatusSignal<Voltage> appliedVoltsSignal;
  // private final StatusSignal<Current> currentSignal;

  // private final Debouncer encoderConnectedDebounce = new Debouncer(0.5);

  private final DigitalInput coralSensor;

  /**
   * Constructs an IntakeIOTalonFX.
   *
   * @param motorId The ID of the motor.
   */
  public IntakeIOTalonFX(int coralSensorChannel) {
    // motor = new TalonFX(motorId);
    coralSensor = new DigitalInput(coralSensorChannel);

    // // Configure motor
    // TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    // motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Normal
    // direction
    // motor.getConfigurator().apply(motorConfig);

    // // Use the TalonFX's built-in relative encoder
    // rpmSignal = motor.getVelocity();
    // appliedVoltsSignal = motor.getMotorVoltage();
    // currentSignal = motor.getStatorCurrent();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Refresh signals
    // var motorStatus = BaseStatusSignal.refreshAll(rpmSignal);

    // Update inputs
    // inputs.intakeEncoderConnected = encoderConnectedDebounce.calculate(motorStatus.isOK());
    // inputs.intakeRPM = rpmSignal.getValueAsDouble();
    // inputs.intakeAppliedVolts = appliedVoltsSignal.getValueAsDouble();
    // inputs.intakeCurrentAmps = currentSignal.getValueAsDouble();

    // Commented out for now
    inputs.coralDetected = coralSensor.get(); // Coral detected if the sensor is triggered
    // (active low)
  }

  @Override
  public void stop() {
    // motor.setControl(neutralOut);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    // motor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setRPM(double rpm) {
    // motor.set(-rpm);
  }
}
