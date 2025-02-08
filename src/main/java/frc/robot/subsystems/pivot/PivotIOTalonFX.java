package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import frc.robot.subsystems.pivot.PivotIO.PivotIOInputs;

public class PivotIOTalonFX implements PivotIO {

  public final SparkFlex pivotMotor;
  public final RelativeEncoder pivotEncoder;
  public final SparkMaxConfig pivotConfig;
  public final SparkClosedLoopController pivotPIDController;
  private final Alert encoderDisconnectedAlert;

  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);

  private final StatusSignal<Angle> encoderPositionSignal;
  private final StatusSignal<Voltage> motorAppliedVoltsSignal;
  private final StatusSignal<Current> motorCurrentSignal;

  private final Debouncer encoderConnectedDebounce = new Debouncer(0.5);
  private final NeutralOut neutralOut = new NeutralOut();


 public PivotIOTalonFX(int MOTOR_ID) {
        this.pivotMotor = new SparkFlex(MOTOR_ID, SparkFlex.MotorType.kBrushless);
        this.pivotEncoder = pivotMotor.getEncoder();
        this.pivotPIDController = pivotMotor.getClosedLoopController();
        this.encoderDisconnectedAlert = new Alert("Pivot encoder disconnected!", Alert.AlertType.kError);
        this.pivotConfig = new SparkMaxConfig();

        pivotConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        pivotConfig.encoder
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
        pivotConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(1.0, 0.0, 0.0)
            .outputRange(-1, 1)
            .velocityFF(1/565);
            
    
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // // Signals
        // encoderPositionSignal = pivotEncoder.getPosition();
        // motorAppliedVoltsSignal = pivotMotor.getBusVoltage();
        // motorCurrentSignal = elevatorMotor.getStatorCurrent();
    }



    @Override
    public void updateInputs(PivotIOInputs inputs) {
    // Refresh signals
    var encoderStatus = BaseStatusSignal.refreshAll(encoderPositionSignal);

    // Update elevator inputs
    inputs.pivotEncoderConnected = encoderConnectedDebounce.calculate(encoderStatus.isOK());
    inputs.pivotPosition = encoderPositionSignal.getValueAsDouble();
    inputs.pivotAppliedVolts = motorAppliedVoltsSignal.getValueAsDouble();
    inputs.pivotCurrentAmps = motorCurrentSignal.getValueAsDouble();
  }
//   @Override
//   public void setPivotPosition(PivotIOInputs.PivotPositions state) {
//     double desiredPosition = state.getPivotPosition();

//         pivotSetpoint = pivotMotor.calculate(kDt,motor1Setpoint,motor1Goal);
//         pivotPIDController.setReference(currentPosition, CANSparkFlex.ControlType.kPosition);
    
//   }
}
