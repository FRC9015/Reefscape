package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  public final PneumaticHub revPH;
  public final Solenoid solenoid1, solenoid2, solenoid3;

  public Climber(int portID) {
    this.revPH = new PneumaticHub(portID);
    this.solenoid1 = revPH.makeSolenoid(1);
    this.solenoid2 = revPH.makeSolenoid(4);
    this.solenoid3 = revPH.makeSolenoid(0);

    this.setDefaultCommand(defaultCommand());
  }

  public void extend1() {
    this.solenoid1.set(true);
  }

  public void retract1() {
    this.solenoid1.set(false);
  }

  // Create commands for unwinding and retracting
  public Command extendCommand1() {
    return run(this::extend1);
  }

  public Command retractCommand1() {
    return run(this::retract1);
  }

  public void extend2() {
    this.solenoid2.set(true);
    // this.solenoid3.set(false);
  }

  public void retract2() {
    this.solenoid2.set(false);
    this.solenoid3.set(true);
  }

  public void default1() {
    this.solenoid1.set(false);
    this.solenoid2.set(true);
  }

  public void goUp() {
    retract2();
    extend1();
  }

  public Command upGo() {
    return run(this::goUp);
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
    this.revPH.enableCompressorDigital();
  }
}
