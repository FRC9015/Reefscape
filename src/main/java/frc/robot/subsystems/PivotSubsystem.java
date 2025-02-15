package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
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
    
    //makes motors
    public final SparkFlex pivotMotor1 = new SparkFlex(61, SparkLowLevel.MotorType.kBrushless);
    public final SparkFlex pivotMotor2 = new SparkFlex(62, SparkLowLevel.MotorType.kBrushless);

    //gets encoders
    public final RelativeEncoder pivotEncoder = pivotMotor1.getEncoder();
    

    //makes PID for motors
    private final PIDController pid = new PIDController(1, 0, 0);
    public final SparkMaxConfig climberConfig;

    //motion profiling
    private final TrapezoidProfile pivot1Profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(3.0, 1.0));
    private final TrapezoidProfile pivot2Profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(3.0, 1.0));
    TrapezoidProfile.State motor1Setpoint = new TrapezoidProfile.State();
    TrapezoidProfile.State motor2Setpoint = new TrapezoidProfile.State();
    TrapezoidProfile.State motor1Goal = new TrapezoidProfile.State();
    TrapezoidProfile.State motor2Goal = new TrapezoidProfile.State();


    private double currentPosition = 0;
    public PivotSubsystem(){

        //sets PID values of both controllers
        this.climberConfig = new SparkMaxConfig();

    climberConfig.inverted(true).idleMode(IdleMode.kBrake);
    climberConfig.encoder.positionConversionFactor(1).velocityConversionFactor(15);

    // Configure the motor
    climberConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    climberConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(2, 0.0, 0.0)
        .outputRange(0, 1)
        .velocityFF(1.0 / 565.0);


        pivotMotor2.follow(pivotMotor1,true);      
;
    }

    public Command raisePivot(){
        return run(
                this::movePivotUp);
    }
    
    public Command lowerPivot(){
        return run(
                this::movePivotDown);
    }
    public Command movePivotToIntake(){
        return this.runOnce(this::intake);
    }

    public Command movePivotToSubWooferAuto(){
        return this.runOnce(this::subWoofer);
    }

    public Command movePivotToSubWoofer(){
        return startEnd(
            this::subWoofer,
            this::intake
        );
    }   
    

    public Command printPivotAngle(){
        return new InstantCommand(
            () -> System.out.println("Current Pivot Position: " 
            + currentPosition)
            );
    }

    //moves pivot up
    private void movePivotUp(){
        currentPosition += 0.005;
    }

    //moves pivot
    private void movePivotDown(){
        currentPosition -= 0.005;
    }

    //uses SparkMax PID to set the motors to a position
    public void intake(){

       
        currentPosition = 0.24;
    }

    //uses SparkMax PID to set the motors to a position
    public void subWoofer(){
        currentPosition = 0;
        
    }

    //uses SparkMax PID to set the motors to a position


    public void setCurrentPosition(double setPoint){

        Logger.recordOutput("Pivot/AutoAim/SetPoint", setPoint);
        currentPosition = MathUtil.clamp(setPoint, 0, 1.3);
    }





     

    @Override
    public void periodic(){
        //puts values on dashboard
        SmartDashboard.putNumber("pivot Position", pivotEncoder.getPosition());

        double kDt = 0.02;
        motor1Setpoint = pivot1Profile.calculate(kDt,motor1Setpoint,motor1Goal);
        motor2Setpoint = pivot2Profile.calculate(kDt,motor2Setpoint,motor2Goal);
        pivotMotor1.set(MathUtil.clamp(pid.calculate(pivotEncoder.getPosition()), -0.9, 0.9));
       
        
        
    }
}
