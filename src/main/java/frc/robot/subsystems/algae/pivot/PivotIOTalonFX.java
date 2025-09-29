package frc.robot.subsystems.algae.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.subsystems.algae.pivot.PivotIO.PivotIOInputs.PivotPosition;

public class PivotIOTalonFX implements PivotIO {

  private TalonFX pivotMotor; // ID 7
  private final VoltageOut voltageOut = new VoltageOut(0.0);
  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0);
  private final StatusSignal<Angle> motorPosition;

  public PivotIOTalonFX(int moterID) {

    this.pivotMotor = new TalonFX(moterID, "CANivore");
    TalonFXConfiguration motorConfig =
        new TalonFXConfiguration()
            .withSlot0(GroundIntakeConstants.GROUND_CONFIGS)
            .withMotionMagic(GroundIntakeConstants.GROUND_MAGIC_CONFIGS)
            .withFeedback(GroundIntakeConstants.GROUND_FEEDBACK_CONFIGS);

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    pivotMotor.getConfigurator().apply(motorConfig);
    motorPosition = pivotMotor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, motorPosition);
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
  public void setPivotPosition(double angle) {
    // pivotMotor.setVoltage(MathUtil.clamp(value, -8, 8));
    if (angle < GroundIntakeConstants.maxPosition) {
      angle = GroundIntakeConstants.maxPosition;
    }
    if (angle > GroundIntakeConstants.minPosition) {
      angle = GroundIntakeConstants.minPosition;
    }
    pivotMotor.setControl(motionMagicVoltage.withPosition(angle));
  }

  @Override
  public void pivotUp(double speed) {
    pivotMotor.setVoltage(MathUtil.clamp(speed, -12, 12));
  }

  @Override
  public void pivotDown(double speed) {
    pivotMotor.setVoltage(MathUtil.clamp(speed, -12, 12));
  }
}
