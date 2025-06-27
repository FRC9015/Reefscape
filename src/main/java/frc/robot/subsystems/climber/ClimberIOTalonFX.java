package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** the. */
public class ClimberIOTalonFX implements ClimberIO {

  public final TalonFX topMotor, climbMotor1, climbMotor2;
  public StatusSignal<Voltage> motorVolts;
  public StatusSignal<Current> motorAmps;
  public StatusSignal<AngularVelocity> motorRPM;

  // private final NeutralOut neutralOut = new NeutralOut();

  /**
   * Constructs an IntakeIOTalonFX.
   *
   * @param motorID The ID of the motor.
   */
  public ClimberIOTalonFX(int topMotorID, int climbID1, int climbID2) {
    topMotor = new TalonFX(topMotorID);
    climbMotor1 = new TalonFX(climbID1);
    climbMotor2 = new TalonFX(climbID2);

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

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, motorVolts, motorAmps, motorRPM);
    ParentDevice.optimizeBusUtilizationForAll(topMotor);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(motorVolts, motorAmps, motorRPM);

    inputs.climberAppliedVolts = motorVolts.getValueAsDouble();
    inputs.climberCurrentAmps = motorAmps.getValueAsDouble();
    inputs.climberPosition = getPosition();
    inputs.climberRPM = motorRPM.getValueAsDouble();
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
    climbMotor1.setVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
    climbMotor2.setVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
  }

  public double getPosition() {
    return topMotor.getPosition().getValueAsDouble();
  }
}
