// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** The EndEffector subsystem controls the end effector mechanism. */
public class EndEffector extends SubsystemBase {
  private final EndEffectorIO io;
  private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();
  private final Alert encoderDisconnectedAlert;
  private final Alert coralDetectionAlert;

  /**
   * Constructs an EndEffector subsystem.
   *
   * @param io The input/output interface for the end effector.
   */
  public EndEffector(EndEffectorIO io) {
    this.io = io;
    encoderDisconnectedAlert = new Alert("End effector encoder disconnected!", AlertType.kError);
    coralDetectionAlert = new Alert("Coral detected!", AlertType.kInfo);
  }

  /** Periodically updates the end effector's state and logs inputs. */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("EndEffector", inputs);

    // Update alerts
    encoderDisconnectedAlert.set(!inputs.endEffectorEncoderConnected);
    coralDetectionAlert.set(inputs.coralDetected);
  }

  /**
   * Sets the end effector's RPM.
   *
   * @param rpm The desired RPM for the end effector.
   */
  public void setRPM(double rpm) {
    io.setRPM(rpm);
    Logger.recordOutput("EndEffector/SetRPM", rpm);
  }

  /** Stops the end effector. */
  public void stop() {
    io.stop();
    Logger.recordOutput("EndEffector/Stopped", true);
  }

  /**
   * Enables or disables brake mode for the end effector motors.
   *
   * @param enable True to enable brake mode, false for coast mode.
   */
  public void setBrakeMode(boolean enable) {
    io.setBrakeMode(enable);
    Logger.recordOutput("EndEffector/BrakeMode", enable);
  }

  /**
   * Returns whether coral is currently detected.
   *
   * @return True if coral is detected, false otherwise.
   */
  public boolean isCoralDetected() {
    return inputs.coralDetected;
  }

  /**
   * Returns the current RPM of the end effector.
   *
   * @return The RPM of the end effector.
   */
  public double getRPM() {
    return inputs.endEffectorRPM;
  }

  /**
   * Returns the current voltage applied to the end effector motors.
   *
   * @return The applied voltage.
   */
  public double getAppliedVolts() {
    return inputs.endEffectorAppliedVolts;
  }

  /**
   * Returns the current drawn by the end effector motors.
   *
   * @return The current in amps.
   */
  public double getCurrentAmps() {
    return inputs.endEffectorCurrentAmps;
  }

  /**
   * Runs the end effector at the specified RPM.
   *
   * @param rpm The desired RPM for the end effector.
   * @return A command that runs the end effector.
   */
  public Command runEffector(double rpm) {
    return this.startEnd(() -> setRPM(-rpm), () -> stop());
  }

  /**
   * Runs the end effector in reverse at the specified RPM.
   *
   * @param rpm The desired RPM for the end effector.
   * @return A command that runs the end effector in reverse.
   */
  public Command runEffectorReverse(double rpm) {
    return this.startEnd(() -> setRPM(rpm), () -> stop());
  }
}
