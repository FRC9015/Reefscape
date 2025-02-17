package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  public final SparkFlex climberMotor;
  public final SparkMaxConfig climberConfig;

  // Constants for motor speed (tune these for your robot)
  private static final double UNWIND_SPEED = -0.8; // Extend (down)
  private static final double RETRACT_SPEED = 0.8; // Pull up

  public Climber(int motorID) {
    this.climberMotor = new SparkFlex(motorID, SparkFlex.MotorType.kBrushless);
    this.climberConfig = new SparkMaxConfig();

    climberConfig.inverted(true).idleMode(IdleMode.kBrake);
    climberConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

    // Configure the motor
    climberConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    climberConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(2, 0.0, 0.0)
        .outputRange(0, 1)
        .velocityFF(1.0 / 565.0);

    // Zero the encoder
    climberMotor.getEncoder().setPosition(0);
  }

  public void unwind() {
    climberMotor.set(UNWIND_SPEED);
  }

  public void retract() {
    climberMotor.set(RETRACT_SPEED);
  }

  public void stop() {
    climberMotor.set(0);
  }

  // Create commands for unwinding and retracting
  public Command unwindCommand() {
    return run(this::unwind).finallyDo((interrupted) -> stop());
  }

  public Command retractCommand() {
    return run(this::retract).finallyDo((interrupted) -> stop());
  }

  @Override
  public void periodic() {
    // Display climber position & status on SmartDashboard
    SmartDashboard.putNumber("Climber Position", climberMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Climber Current", climberMotor.getOutputCurrent());
  }
}
