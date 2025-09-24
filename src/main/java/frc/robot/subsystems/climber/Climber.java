package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;
import org.littletonrobotics.junction.Logger;

/** Creates the Climber subsystem class. */
public class Climber extends SubsystemBase {

  public final ClimberIO io;
  public ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private final PIDController pidController;

  // Pivot PID constants
  private static final double kP = 2.0;
  private static final double kI = 0.0;
  private static final double kD = 0.02;
  private static final double kToleranceMeters = 0.01;

  /** Constructor for the Climber Class. */
  public Climber(int portID, ClimberIO io) {
    this.io = io;
    this.pidController = new PIDController(kP, kI, kD);
    this.pidController.setTolerance(kToleranceMeters);
    this.setDefaultCommand(extendCommand2());
  }

  /** Locks the servos for the ramp.*/

  public void extend2() {
    io.servoLock();
  }

  /** Retracts the servos for the ramp. */
  public void retract2() {
    io.servoUnlock();
  }

  /** Sets the position of the climber using voltage control. 
   * Logs climber setpoint, output, and position. 
   *
   *@param 
   *  state the desired state of the climber.*/
  public void setPreset(ClimberIOInputs.ClimberPositions state) {
    pidController.setSetpoint(state.getClimberPosition());
    double output = pidController.calculate(inputs.climberPosition);
    io.setClimbRPM(output);
    io.updateInputs(inputs);

    Logger.recordOutput("Climber/Setpoint", state.getClimberPosition());
    Logger.recordOutput("Climber/Output", output);
    Logger.recordOutput("Climber/PositionInput", inputs.climberPosition);
  }

  /** Runs a command to set the position of the climber using voltage control.
   *  Ends once the PID has reached the setpoint.
   *
   * @param 
   *  state the desired state of the climber. 
   * 
   * @return
   *      A run Command that runs the PID until the controller has reached the desired setpoint. */
  public Command executePreset(ClimberIOInputs.ClimberPositions state) {
    Logger.recordOutput("Climber/State", state);
    return run(() -> this.setPreset(state)).until(() -> pidController.atSetpoint());
  }

  /** Creates a runOnce command to retract the ramp servos.*/
  public Command upGo() {
    return runOnce(this::retract2);
  }

  /** Sends the climber outwards at 10 volts. */
  public Command up() {
    return this.startEnd(() -> io.setClimbRPM(10), () -> io.setClimbRPM(0));
  }

  /** Brings the climber in at 10 volts. */
  public Command down() {
    return this.startEnd(() -> io.setClimbRPM(-10), () -> io.setClimbRPM(0));
  }

  /** Spins the climb wheels. */
  public Command topMotor() {
    return this.startEnd(() -> io.setTopRPM(6), () -> io.setTopRPM(0));
  }

  /** Creates run command to retract the ramp servos. */
  public Command extendCommand2() {
    return run(this::extend2);
  }

  /** Creates run command to retract the ramp servos. */
  public Command retractCommand2() {
    return run(this::retract2); 
  }

  /** Sets climb voltage to 0.*/
  public Command stopClimb() {
    return this.runOnce(() -> io.setClimbRPM(0.0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }
}
