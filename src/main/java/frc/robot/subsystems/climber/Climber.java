package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  public final PneumaticHub revPH;
  public final Solenoid solenoid1;

  public Climber(int portID) {
    this.revPH = new PneumaticHub(portID);
    this.solenoid1 = revPH.makeSolenoid(0);
    this.setDefaultCommand(retractCommand());
  }

  public void extend() {
    this.solenoid1.set(true);
  }

  public void retract() {
    this.solenoid1.set(false);
  }

  // Create commands for unwinding and retracting
  public Command extendCommand() {
    return run(this::extend);
  }

  public Command retractCommand() {
    return run(this::retract);
  }

  @Override
  public void periodic() {
    this.revPH.enableCompressorDigital();
  }
}
