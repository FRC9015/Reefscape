package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** the. */
public class ClimberIOTalonFX implements ClimberIO {

  public final TalonFX topMotor, climbMotor1, climbMotor2;
  public final Servo servo1, servo2;
  public StatusSignal<Voltage> motorVolts;
  public StatusSignal<Current> motorAmps;
  public StatusSignal<AngularVelocity> motorRPM;
  public StatusSignal<Angle> motorPosition;
  private LoggedNetworkNumber minPosition = new LoggedNetworkNumber("/Tunning/minPosition", 0.0);
  private LoggedNetworkNumber maxPosition = new LoggedNetworkNumber("/Tunning/maxPOsition", 1.0);
  private final VoltageOut voltageOut = new VoltageOut(0.0);
  private final MotionMagicVoltage positionVoltage = new MotionMagicVoltage(0.0);

  // private final NeutralOut neutralOut = new NeutralOut();

  /**
   * Constructs an IntakeIOTalonFX.
   *
   * @param motorID The ID of the motor.
   */
  public ClimberIOTalonFX(
      int topMotorID, int climbID1, int climbID2, int servoCannel1, int servoCannel2) {
    topMotor = new TalonFX(topMotorID);
    climbMotor1 = new TalonFX(climbID1);
    climbMotor2 = new TalonFX(climbID2);
    servo1 = new Servo(servoCannel1);
    servo2 = new Servo(servoCannel2);

    // Configure motor
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    TalonFXConfiguration motorConfig2 = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Configure the integrated encoder (default settings should work)
    topMotor.getConfigurator().apply(motorConfig);
    climbMotor1.getConfigurator().apply(motorConfig);
    climbMotor2.getConfigurator().apply(motorConfig2);
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorVolts = topMotor.getMotorVoltage();
    motorAmps = topMotor.getStatorCurrent();
    motorRPM = topMotor.getVelocity();
    motorPosition = climbMotor1.getPosition();
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, motorVolts, motorAmps, motorRPM);
    ParentDevice.optimizeBusUtilizationForAll(topMotor);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(motorVolts, motorAmps, motorRPM, motorPosition);
    inputs.climberAppliedVolts = motorVolts.getValueAsDouble();
    inputs.climberCurrentAmps = motorAmps.getValueAsDouble();
    // inputs.climberPosition = getPosition();
    inputs.climberRPM = motorRPM.getValueAsDouble();
    inputs.servoPosition = servo1.getPosition();
    inputs.climberPosition = motorPosition.getValueAsDouble();
  }

  @Override
  public void stop() {}

  @Override
  public void setBrakeMode(boolean enable) {}

  @Override
  public void setTopRPM(double voltage) {
    topMotor.setVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
  }

  @Override
  public void setClimbRPM(double voltage) {
    climbMotor1.setVoltage(MathUtil.clamp(voltage, -12, 12));
    // climbMotor2.setVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
  }

  @Override
  public void setClimbPosition(double position) {
    climbMotor1.setControl(positionVoltage.withPosition(position));
  }

  public double getPosition() {
    return climbMotor1.getPosition().getValueAsDouble();
  }

  @Override
  public void servoUnlock() {
    servo1.setPosition(0.2);
    servo2.setPosition(0.2);
  }

  @Override
  public void servoLock() {
    servo1.setPosition(0.7);
    servo2.setPosition(0.7);
  }
}
