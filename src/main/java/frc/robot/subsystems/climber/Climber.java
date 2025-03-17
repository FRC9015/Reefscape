package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private ClimberIO io;

  public Climber(ClimberIO io) {
    this.io = io;
  }

  public Command setSpeed(double rpm) {
    return run(() -> io.setRPM(rpm));
  }

  @Override
  public void periodic() {
    io.getPosition();
  }
}
