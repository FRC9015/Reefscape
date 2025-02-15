
package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;




import org.littletonrobotics.junction.Logger;


public class ShooterSubsystem extends SubsystemBase {
    private final DigitalInput speakerSensor = new DigitalInput(4);
    
    private final double motorMaxFreeSpeed = 6784;//RPM
    private boolean shooterIsRunning = false;
    private boolean idleMode = false;
    private PIDController speakerPIDTop = new PIDController((25/motorMaxFreeSpeed),0,0);
    private PIDController speakerPIDBottom = new PIDController((25/motorMaxFreeSpeed),0,0);
    private final SparkFlex speakerMotorTop = new SparkFlex(51,
            MotorType.kBrushless);
    private final SparkFlex speakerMotorBottom = new SparkFlex(52,
            MotorType.kBrushless);


    RelativeEncoder speakerMotorTopEncoder = speakerMotorTop.getEncoder();
    RelativeEncoder speakerMotorBottomEncoder = speakerMotorBottom.getEncoder();

    public ShooterSubsystem() {
     
    }

    public Command shootNoteToSpeaker() {
        return this.startEnd(
                this::setSpeakerShooterMotorSpeeds,
                this::setIdleShooterSpeeds);
    }

    public Command stopShooter() {
        return this.runOnce(
                this::stopSpeakerShooterMotors);
    }

    public Command shooterBackward(){
        return this.startEnd(
                this::backwardsShooter,
                this::setIdleShooterSpeeds);
    }


    public Command enableIdleMode(){
        return this.runOnce(this::setIdleShooterSpeeds).onlyWhile(this::getShooterSensor);
    }



    public void setIdleShooterSpeeds() {
        speakerPIDTop.setSetpoint(0.1 * motorMaxFreeSpeed);
        speakerPIDBottom.setSetpoint(0.1 * motorMaxFreeSpeed);
        shooterIsRunning = true;
        idleMode = true;
    }


    public void setSpeakerShooterMotorSpeeds(){
        double shooterSpeeed = 0.2;
        speakerPIDTop.setSetpoint(0.2 * motorMaxFreeSpeed);
        speakerPIDBottom.setSetpoint(shooterSpeeed * motorMaxFreeSpeed);
        shooterIsRunning = true;
        idleMode = false;
    }


    public void stopSpeakerShooterMotors() {
        speakerMotorTop.stopMotor();
        speakerMotorBottom.stopMotor();
        shooterIsRunning = false;
        idleMode = false;
    }


    private void backwardsShooter(){
        speakerPIDTop.setSetpoint(-0.4 * motorMaxFreeSpeed);
        speakerPIDBottom.setSetpoint(-0.4 * motorMaxFreeSpeed);
        shooterIsRunning = true;
        idleMode = false;
    }


    public boolean getShooterSensor() {
        return !speakerSensor.get();
    }

    public boolean shooterIsReady(){
        return (speakerPIDTop.atSetpoint() && speakerPIDBottom.atSetpoint() && shooterIsRunning && !idleMode);
    }


    @Override
    public void periodic() {
        if (shooterIsRunning) {
            double outputTop = MathUtil.clamp(speakerPIDTop.calculate(speakerMotorTopEncoder.getVelocity())+speakerPIDTop.getSetpoint()*13/motorMaxFreeSpeed,-10,10);
            double outputBottom = MathUtil.clamp(speakerPIDBottom.calculate(speakerMotorBottomEncoder.getVelocity())+speakerPIDBottom.getSetpoint()*13/motorMaxFreeSpeed,-10,10);

            speakerMotorTop.setVoltage(outputTop);
            speakerMotorBottom.setVoltage(outputBottom);
            // If Issues Persist, Log Data With the Following Code:
           Logger.recordOutput("Shooter/TopMotor/Speed",speakerMotorTopEncoder.getVelocity());
           Logger.recordOutput("Shooter/BottomMotor/Speed",speakerMotorBottomEncoder.getVelocity());

        }
        if (idleMode && !getShooterSensor()){
            stopSpeakerShooterMotors();
        }
        SmartDashboard.putBoolean("Shooter Sensor", getShooterSensor());

        Logger.recordOutput("Sensors/ShooterSensor", getShooterSensor());
    }


    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
