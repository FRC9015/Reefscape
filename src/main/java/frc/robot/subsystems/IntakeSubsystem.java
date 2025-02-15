package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalOutput;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;



public class IntakeSubsystem extends SubsystemBase {
    private final DigitalOutput handoffSensor = new DigitalOutput(2);
    public boolean handoff = true;
    private SparkFlex[] intakeMotors = new SparkFlex[]{
        new SparkFlex(41, MotorType.kBrushless),
    };
    private final SparkFlex handoffMotor = new SparkFlex(43, MotorType.kBrushless);
    RelativeEncoder handoffMotorEncoder = handoffMotor.getEncoder();

    public IntakeSubsystem(){
      
        handoff = true;
    }

    public Command intakeNote(){
        return this.startEnd(
                this::setIntakeMotorSpeeds,
                this::stopIntakeMotors
        );
    }

    public Command runIntake(){
        return this.run(this::setIntakeMotorSpeeds);
    }
    
    public Command autoIntakeNote(){
        return new SequentialCommandGroup(
                new InstantCommand(this::setIntakeMotorSpeeds),
                new WaitCommand(1),
                new InstantCommand(this::stopIntakeMotors));
    }

    public Command outtakeNote(){
        return this.startEnd(
                this::setReverseIntakeMotorSpeeds,
                this::stopIntakeMotors
        );
    }

    public Command autoOuttakeNote(){
        return new SequentialCommandGroup(
                new InstantCommand(this::setReverseIntakeMotorSpeeds),
                new WaitCommand(3),
                new InstantCommand(this::stopIntakeMotors));
    }

    public Command stopIntake(){
        return this.runOnce(
            this::stopIntakeMotors
        );
    }


    private void setIntakeMotorSpeeds(){
        double motorSpeed = -0.8;
        for (SparkFlex motor : intakeMotors) {
            motor.set(motorSpeed);
        }
        handoffMotor.set(-motorSpeed);
    }
    private void setReverseIntakeMotorSpeeds(){
        double motorSpeed = 0.8;
        for(SparkFlex motor:intakeMotors){
            motor.set(motorSpeed);
        }
        handoffMotor.set(-motorSpeed);
    }
    private void stopIntakeMotors(){
        for (SparkFlex motor:intakeMotors){
            motor.stopMotor();
        }
        handoffMotor.stopMotor();
    }

    public boolean getHandoffStatus(){

        return handoffSensor.get();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Intake Sensor",handoffSensor.get());
        Logger.recordOutput("Sensors/HandOffSensor", handoffMotor.get());

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}
