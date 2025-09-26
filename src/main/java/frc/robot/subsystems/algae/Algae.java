package frc.robot.subsystems.algae;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Algae extends SubsystemBase {
  private final AlgaeIO io;
  private final AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();
  private final Alert encoderDisconnectedAlert;
  private final PIDController pidController;

  // Algae PID constants
  private static final double kP = 10.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kToleranceMeters = 0.01; // Acceptable position error in meters

  public Algae(AlgaeIO io) {
    this.io = io;
    this.pidController = new PIDController(kP, kI, kD);
    this.pidController.setTolerance(kToleranceMeters);
    encoderDisconnectedAlert = new Alert("Algae encoder disconnected!", AlertType.kError);
  }

  /** Periodically updates the algae's state and logs inputs. */
  public void periodic() {
    io.updateInputs(inputs);
    encoderDisconnectedAlert.set(!inputs.algaeEncoderConnected);
    Logger.recordOutput("Algae/velocity", inputs.algaeRPM);
    Logger.recordOutput("Algae/current", inputs.algaeCurrentAmps);
  }

  public Command setSpeed(double rpm) {
    return run(() -> io.setRPM(rpm));
  }

  public boolean isStalled() {
    // these need to be set basaed on the motors need to be determined in real time.
    double velocityThreshold = 0.0;
    double currentThreshold = 0.0;

    if (Math.abs(inputs.algaeRPM) < velocityThreshold
        && inputs.algaeCurrentAmps > currentThreshold) {
      return true;
    }
    return false;
  }
}
