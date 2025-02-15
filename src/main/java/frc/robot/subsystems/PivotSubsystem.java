package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class PivotSubsystem extends SubsystemBase {

    // Create motors
    public final SparkFlex pivotMotor1 = new SparkFlex(61, SparkLowLevel.MotorType.kBrushless);
    public final SparkFlex pivotMotor2 = new SparkFlex(62, SparkLowLevel.MotorType.kBrushless);

    // Get encoder from leader motor
    public final RelativeEncoder pivotEncoder = pivotMotor1.getEncoder();

    // Create PID controller instance
    private final PIDController pid = new PIDController(1, 0, 0);
    
    // Create motor configuration objects
    public final SparkMaxConfig pivotMotor1Config;
    public final SparkMaxConfig pivotMotor2Config;

    // Motion profiling
    private final TrapezoidProfile pivot1Profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(3.0, 1.0));
    private final TrapezoidProfile pivot2Profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(3.0, 1.0));
    TrapezoidProfile.State motor1Setpoint = new TrapezoidProfile.State();
    TrapezoidProfile.State motor2Setpoint = new TrapezoidProfile.State();
    TrapezoidProfile.State motor1Goal = new TrapezoidProfile.State();
    TrapezoidProfile.State motor2Goal = new TrapezoidProfile.State();

    private double currentPosition = 0;

    public PivotSubsystem() {
        // Initialize configuration for pivotMotor1 (leader)
        pivotMotor1Config = new SparkMaxConfig();
        pivotMotor1Config
            .inverted(false) // Leader motor not inverted
            .idleMode(IdleMode.kBrake)
            .encoder
                .positionConversionFactor(1) // Adjust as needed
                .velocityConversionFactor(15); // Adjust as needed

        pivotMotor1Config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(2.0, 0.0, 0.0) // Adjust PID gains
            .outputRange(-1.0, 1.0)
            .velocityFF(1.0 / 565.0); // Adjust as needed

        // Initialize configuration for pivotMotor2 (follower)
        pivotMotor2Config = new SparkMaxConfig();
        pivotMotor2Config
            .apply(pivotMotor1Config) // Apply global config from leader
            .follow(pivotMotor1); // Set as a follower

        // Apply configurations to both motors
        pivotMotor1.configure(pivotMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotMotor2.configure(pivotMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command raisePivot() {
        return run(this::movePivotUp);
    }

    public Command lowerPivot() {
        return run(this::movePivotDown);
    }

    public Command movePivotToIntake() {
        return this.runOnce(this::intake);
    }

    public Command movePivotToSubWooferAuto() {
        return this.runOnce(this::subWoofer);
    }

    public Command movePivotToSubWoofer() {
        return startEnd(
            this::subWoofer,
            this::intake
        );
    }   
    
    public Command printPivotAngle() {
        return new InstantCommand(
            () -> System.out.println("Current Pivot Position: " + currentPosition)
        );
    }

    // Moves the pivot up by increasing the current position
    private void movePivotUp() {
        currentPosition = MathUtil.clamp(currentPosition + 0.005, 0, 1.3);
    }

    // Moves the pivot down by decreasing the current position
    private void movePivotDown() {
        currentPosition = MathUtil.clamp(currentPosition - 0.005, 0, 1.3);
    }

    // Sets the pivot position to the intake position using SparkMax PID
    public void intake() {
        currentPosition = 0.24;
    }

    // Sets the pivot position to the subWoofer position using SparkMax PID
    public void subWoofer() {
        currentPosition = 0;
    }

    // Allows manual adjustment of the pivot's current position
    public void setCurrentPosition(double setPoint) {
        Logger.recordOutput("Pivot/AutoAim/SetPoint", setPoint);
        currentPosition = MathUtil.clamp(setPoint, 0, 1.3);
    }

    @Override
    public void periodic() {
        // Output the current pivot position to the dashboard
        SmartDashboard.putNumber("Pivot Position", pivotEncoder.getPosition());

        double kDt = 0.02;
        motor1Setpoint = pivot1Profile.calculate(kDt, motor1Setpoint, motor1Goal);
        motor2Setpoint = pivot2Profile.calculate(kDt, motor2Setpoint, motor2Goal);
        
        // Apply the PID calculation to pivotMotor1; pivotMotor2 will follow it
        pivotMotor1.set(MathUtil.clamp(pid.calculate(pivotEncoder.getPosition()), -0.9, 0.9));
    }
}
