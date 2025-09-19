package frc.robot.subsystems.algae.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.algae.pivot.PivotIO.PivotIOInputs.PivotPosition;

public class PivotIOTalonFX implements PivotIO {

  private TalonFX pivotMotor; // ID 7
  private final VoltageOut voltageOut = new VoltageOut(0.0);
  private final StatusSignal<Angle> motorPosition;

  public PivotIOTalonFX(int moterID) {

    this.pivotMotor = new TalonFX(moterID, "*");
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    pivotMotor.getConfigurator().apply(motorConfig);
    motorPosition = pivotMotor.getPosition();
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(motorPosition);

    inputs.pivotState = PivotPosition.Default;
    inputs.pivotAppliedVolts = pivotMotor.getMotorVoltage().getValueAsDouble();
    inputs.pivotAtSetpoint = false;
    inputs.pivotCurrentAmps = pivotMotor.getStatorCurrent().getValueAsDouble();
    inputs.pivotEncoderConnected = false;
    inputs.pivotPosition = motorPosition.getValueAsDouble();
  }

  @Override
  public void setPivotPosition(double value) {
    pivotMotor.setVoltage(MathUtil.clamp(value, -12, 12));
  }
}
