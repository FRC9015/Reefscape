package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
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

public class ClimberIOTalonFX implements ClimberIO {

  public final TalonFX motor;

  // private final NeutralOut neutralOut = new NeutralOut();

  /**
   * Constructs an IntakeIOTalonFX.
   *
   * @param motorId The ID of the motor.
   */
  public ClimberIOTalonFX(int motorID) {
    motor = new TalonFX(motorID);

    // Configure motor
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Configure the integrated encoder (default settings should work)
    motor.getConfigurator().apply(motorConfig);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberAppliedVolts = 0.0;
    inputs.climberCurrentAmps = 0.0;
    inputs.climberPosition = 0.0;
    inputs.climberRPM = 0.0;
  }

  @Override
  public void stop() {}

  @Override
  public void setBrakeMode(boolean enable) {}

  @Override
  public void setRPM(double voltage) {
    motor.setVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
  }

  @Override
  public void getPosition() {
    Logger.recordOutput("Climber/position", motor.getPosition().toString());
  }
}
