package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  public final ClimberIO io;
  public ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private final PIDController pidController;

  // Pivot PID constants
  private static final double kP = 2.0;
  private static final double kI = 0.0;
  private static final double kD = 0.02;
  private static final double kToleranceMeters = 0.01;

  public Climber(int portID, ClimberIO io) {
    this.io = io;
    this.pidController = new PIDController(kP, kI, kD);
    this.pidController.setTolerance(kToleranceMeters);
  }

  public void extend2() {
    io.servoLock();
  }

  public void retract2() {
    io.servoUnlock();
  }

  public void setPreset(ClimberIOInputs.ClimberPositions state) {
    pidController.setSetpoint(state.getClimberPosition());
    double output = pidController.calculate(inputs.climberPosition);
    io.setClimbRPM(output);
    io.updateInputs(inputs);

    Logger.recordOutput("Climber/Setpoint", state.getClimberPosition());
    Logger.recordOutput("Climber/Output", output);
    Logger.recordOutput("Climber/PositionInput", inputs.climberPosition);
  }

  public Command executePreset(ClimberIOInputs.ClimberPositions state) {
    Logger.recordOutput("Climber/State", state);
    return run(() -> this.setPreset(state)).until(() -> pidController.atSetpoint());
  }

  public Command upGo() {
    return runOnce(this::retract2);
  }

  public Command up() {
    return this.startEnd(() -> io.setClimbRPM(10), () -> io.setClimbRPM(0));
  }

  public Command down() {
    return this.startEnd(() -> io.setClimbRPM(-10), () -> io.setClimbRPM(0));
  }

  public Command topMotor() {
    return this.startEnd(() -> io.setTopRPM(6), () -> io.setTopRPM(0));
  }

  // Create commands for unwinding and retracting
  public Command extendCommand2() {
    return run(this::extend2);
  }

  public Command retractCommand2() {
    return run(this::retract2);
  }

  public Command stopClimb() {
    return this.runOnce(() -> io.setClimbRPM(0.0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }
}
