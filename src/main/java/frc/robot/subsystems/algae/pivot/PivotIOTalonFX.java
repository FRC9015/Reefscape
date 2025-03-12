package frc.robot.subsystems.algae.pivot;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.algae.pivot.PivotIO.PivotIOInputs.PivotPosition;

public class PivotIOTalonFX implements PivotIO {

  private TalonFX pivotMotor; // ID 3
  private final VoltageOut voltageOut = new VoltageOut(0.0);

  public PivotIOTalonFX(int moterID) {

    this.pivotMotor = new TalonFX(moterID);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.pivotState = PivotPosition.Default;
    inputs.pivotAppliedVolts = 0.0;
    inputs.pivotAtSetpoint = false;
    inputs.pivotCurrentAmps = 0.0;
    inputs.pivotEncoderConnected = false;
    inputs.pivotPosition = pivotMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void setPivotPosition(double value) {
    pivotMotor.setControl(voltageOut.withOutput(MathUtil.clamp(value, -12, 12)));
  }
}
