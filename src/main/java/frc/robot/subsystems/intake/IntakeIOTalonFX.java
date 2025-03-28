package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import org.littletonrobotics.junction.Logger;

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

public class IntakeIOTalonFX implements IntakeIO {

  // public final TalonFX motor;
  // private final NeutralOut neutralOut = new NeutralOut();
  // private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

  // private final StatusSignal<AngularVelocity> rpmSignal;
  // private final StatusSignal<Voltage> appliedVoltsSignal;
  // private final StatusSignal<Current> currentSignal;

  // private final Debouncer encoderConnectedDebounce = new Debouncer(0.5);

  private final DigitalInput coralInSensor;
  private final DigitalInput coralSetSensor;
  private final CANrange middleRange;

  private final StatusSignal<Distance> rangeDistance;
  private final StatusSignal<Distance> rangeSTDdevs;

  /**
   * Constructs an IntakeIOTalonFX.
   *
   * @param motorId The ID of the motor.
   */
  public IntakeIOTalonFX(int coralInChannel, int coralSetChannel, int canRangeID1) {
    // motor = new TalonFX(motorId);
    coralInSensor = new DigitalInput(coralInChannel);
    coralSetSensor = new DigitalInput(coralSetChannel);
    middleRange = new CANrange(canRangeID1, "*");

    CANrangeConfiguration rangeConfig = new CANrangeConfiguration();
    // rangeConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 0.0;

    middleRange.getConfigurator().apply(rangeConfig);

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
    rangeDistance = middleRange.getDistance();
    rangeSTDdevs = middleRange.getDistanceStdDev();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Refresh signals
    BaseStatusSignal.refreshAll(rangeDistance, rangeSTDdevs);
    // Update inputs
    // inputs.intakeEncoderConnected = encoderConnectedDebounce.calculate(motorStatus.isOK());
    // inputs.intakeRPM = rpmSignal.getValueAsDouble();
    // inputs.intakeAppliedVolts = appliedVoltsSignal.getValueAsDouble();
    // inputs.intakeCurrentAmps = currentSignal.getValueAsDouble();

    // Commented out for now
    inputs.coralIn = coralInSensor.get(); // Coral detected if the sensor is triggered
    inputs.coralSet = coralSetSensor.get();
    inputs.canRangeSTDdevs = rangeSTDdevs.getValueAsDouble();
    inputs.canRangeDistance = rangeDistance.getValueAsDouble();
    // (active low)
    Logger.recordOutput("coralIn?", !coralInSensor.get());
    Logger.recordOutput("coralSet?", !coralSetSensor.get());
    Logger.recordOutput("getDistance", middleRange.getDistance(true).getValueAsDouble());
    Logger.recordOutput("getSTDdevs", middleRange.getDistanceStdDev(true).getValueAsDouble());
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
