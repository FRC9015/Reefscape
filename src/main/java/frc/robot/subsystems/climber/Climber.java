package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  public final PneumaticHub revPH;
  public final Solenoid solenoid2, solenoid3;
  public final ClimberIO io;

  public Climber(int portID, ClimberIO io) {
    this.revPH = new PneumaticHub(portID);
    this.solenoid2 = revPH.makeSolenoid(4);
    this.solenoid3 = revPH.makeSolenoid(0);
    this.io = io;
    this.setDefaultCommand(defaultCommand());
  }
/**
 * Sets solenoid 2 true, and solenoid 3 false, but solenoid 3 becoming false is not being done.
 */
  public void extend2() {
    this.solenoid2.set(true);
    // this.solenoid3.set(false);
  }
/**
 * Sets solenoid 2 false, and solenoid 3 true.
 */
  public void retract2() {
    this.solenoid2.set(false);
    this.solenoid3.set(true);
  }
/**
 * Sets solenoid 2 true.
 */
  public void default1() {
    this.solenoid2.set(true);
  }
/**
 *  Creates a reference to the method retract2() of the current object (this) and runs it once
 * @see retract2
 */
  public Command upGo() {
    return runOnce(this::retract2);
  }
/**
 * When it starts it sets ClimbRPM to 10, an when it finishes it sets ClimbRPM to 0
 */
  public Command up() {
    return this.startEnd(() -> io.setClimbRPM(10), () -> io.setClimbRPM(0));
  }
/**
 * When it starts it sets ClimbRPM to -10, an when it finishes it sets ClimbRPM to 0
 */
  public Command down() {
    return this.startEnd(() -> io.setClimbRPM(-10), () -> io.setClimbRPM(0));
  }
/**
 * When it starts it sets TopRPM to 6, an when it finishes it sets TopRPM to 0
 */
  public Command topMotor() {
    return this.startEnd(() -> io.setTopRPM(6), () -> io.setTopRPM(0));
  }

  // Create commands for unwinding and retracting
  /**
   * Runs extend2
   * @see extend2
   */
  public Command extendCommand2() {
    return run(this::extend2);
  }
/**
 * Runs retract2
 * @see retract2
 */
  public Command retractCommand2() {
    return run(this::retract2);
  }
/**
 * Runs default 1, is the default command so it runs constantly when not doing something.
 * @see default1
 */
  public Command defaultCommand() {
    return run(this::default1);
  }
/**
 * Enables the digital compressor constantly
 */
  @Override
  public void periodic() {
    this.revPH.enableCompressorDigital();
  }
}
