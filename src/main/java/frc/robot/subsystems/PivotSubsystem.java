package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
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
    public final SparkMaxConfig climberConfig;

    // Motion profiling
    private final TrapezoidProfile pivot1Profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(3.0, 1.0));
    private final TrapezoidProfile pivot2Profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(3.0, 1.0));
    TrapezoidProfile.State motor1Setpoint = new TrapezoidProfile.State();
    TrapezoidProfile.State motor2Setpoint = new TrapezoidProfile.State();
    TrapezoidProfile.State motor1Goal = new TrapezoidProfile.State();
    TrapezoidProfile.State motor2Goal = new TrapezoidProfile.State();

    private double currentPosition = 0;

    public PivotSubsystem() {
        // Initialize configuration for the motor
        this.climberConfig = new SparkMaxConfig();
        climberConfig.inverted(true).idleMode(IdleMode.kBrake);
        climberConfig.encoder.positionConversionFactor(1).velocityConversionFactor(15);

        // Configure the motor settings including encoder conversion factors, PID, and feed-forward
        climberConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(2, 0.0, 0.0)
            .outputRange(0, 1)
            .velocityFF(1.0 / 565.0);

        // Updated follower configuration for REVLib 2025:
        // Set pivotMotor2 to follow pivotMotor1 by using the closed-loop controller's setReference method.
        pivotMotor2.getClosedLoopController().setReference(
            pivotMotor1.getDeviceId(),
            SparkBase.ControlType.
        );
        pivotMotor1.configure(climberConfig, 
        ResetMode.kResetSafeParameters, 
        PersistMode.kPersistParameters);
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
        currentPosition += 0.005;
    }

    // Moves the pivot down by decreasing the current position
    private void movePivotDown() {
        currentPosition -= 0.005;
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
        SmartDashboard.putNumber("pivot Position", pivotEncoder.getPosition());

        double kDt = 0.02;
        motor1Setpoint = pivot1Profile.calculate(kDt, motor1Setpoint, motor1Goal);
        motor2Setpoint = pivot2Profile.calculate(kDt, motor2Setpoint, motor2Goal);
        
        // Apply the PID calculation to pivotMotor1; the follower will mirror its output
        pivotMotor1.set(MathUtil.clamp(pid.calculate(pivotEncoder.getPosition()), -0.9, 0.9));
    }
}
