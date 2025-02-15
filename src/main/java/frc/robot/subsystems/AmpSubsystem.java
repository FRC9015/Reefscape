package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class AmpSubsystem extends SubsystemBase{
        
    private final SparkFlex ampMotorTop = new SparkFlex(53,
            MotorType.kBrushless);
    private final SparkFlex ampMotorBottom = new SparkFlex(54,
            MotorType.kBrushless);

    public AmpSubsystem(){

    }

    public Command ampIntake(){
        return this.startEnd(
                this::setAmpIntakeSpeeds,
                this::stopAmpShooterMotorSpeeds
        );
    }

    public Command shootNoteToAmp() {
        return this.startEnd(
                this::setAmpShooterMotorSpeeds,
                this::stopAmpShooterMotorSpeeds
        );
    }

    public Command stopAmp(){
        return this.runOnce(this::stopAmpShooterMotorSpeeds);
    }

    public void setAmpShooterMotorSpeeds() {
        double motorSpeed = 0.8;// needs to be tuned
        ampMotorTop.set(-motorSpeed);
        ampMotorBottom.set(motorSpeed);
    }

    public void stopAmpShooterMotorSpeeds() {
        ampMotorTop.stopMotor();
        ampMotorBottom.stopMotor();
    }

    public void setAmpIntakeSpeeds(){
        double motorSpeed = 1; //needs to be tuned
        ampMotorTop.set(motorSpeed);
        ampMotorBottom.set(motorSpeed);
    }
}
