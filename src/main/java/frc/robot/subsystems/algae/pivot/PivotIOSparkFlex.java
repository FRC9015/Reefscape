package frc.robot.subsystems.algae.pivot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class PivotIOSparkFlex implements PivotIO {

  public final SparkFlex pivotMotor;
  public final RelativeEncoder pivotEncoder;
  public final SparkMaxConfig pivotConfig;
  public final SparkClosedLoopController pivotPIDController;
  private final TrapezoidProfile pivotProfile;

  TrapezoidProfile.State motorSetpoint = new TrapezoidProfile.State();
  TrapezoidProfile.State motorGoal = new TrapezoidProfile.State();

  public PivotIOSparkFlex(int motorID) {
    this.pivotMotor = new SparkFlex(motorID, SparkFlex.MotorType.kBrushless);
    this.pivotEncoder = pivotMotor.getEncoder();
    this.pivotPIDController = pivotMotor.getClosedLoopController();
    this.pivotProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(3.0, 1.0));
    this.pivotConfig = new SparkMaxConfig();

    pivotConfig.inverted(true).idleMode(IdleMode.kBrake);
    pivotConfig.encoder.positionConversionFactor(60).velocityConversionFactor(1);
    pivotConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(2, 0.0, 0.0)
        .outputRange(0, 1)
        .velocityFF(1.0 / 565.0); // Value received from REV docs:
    // https://docs.revrobotics.com/sites-test-ion/rev-ion-brushless/revlib/closed-loop-control-overview/closed-loop-control-getting-started

    pivotMotor.configure(
        pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setPivotPosition(double value) {
    value = MathUtil.clamp(value, -12, 12);
    pivotPIDController.setReference(value, SparkFlex.ControlType.kVoltage);
  }

  @Override
  public void pivotUp(double speed) {
    pivotMotor.set(speed);
  }

  @Override
  public void pivotDown(double speed) {
    pivotMotor.set(-speed);
  }
}
