package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  public final ClimberIO io;
  public ClimberIOInputsAutoLogged inputs;

  public Climber(int portID, ClimberIO io) {
    // this.revPH = new PneumaticHub(portID);
    // this.solenoid2 = revPH.makeSolenoid(4);
    // this.solenoid3 = revPH.makeSolenoid(0);
    this.io = io;
    this.setDefaultCommand(defaultCommand());
  }

  public void extend2() {
    io.servoOpen();
  }

  public void retract2() {
    io.servoClose();
  }

  public void default1() {
    // this.solenoid2.set(true);
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

  public Command defaultCommand() {
    return run(this::default1);
  }

  @Override
  public void periodic() {
    // this.revPH.enableCompressorDigital();
    // io.updateInputs(inputs);
  }
}
