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

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final Alert encoderDisconnectedAlert;
  private final Alert coralInAlert;
  private final Alert coralSetAlert;

  public Intake(IntakeIO io) {
    this.io = io;
    encoderDisconnectedAlert = new Alert("Intake encoder disconnected!", AlertType.kError);
    coralInAlert = new Alert("Coral detected!", AlertType.kInfo);
    coralSetAlert = new Alert("Coral Set!", AlertType.kInfo);
  }

  /** Periodically updates the intake's state and logs inputs. */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Update alerts
    coralInAlert.set(inputs.coralIn);
    coralSetAlert.set(inputs.coralSet);
  }

  /**
   * Sets the intake's RPM.
   *
   * @param rpm The desired RPM for the intake.
   */
  public void setRPM(double rpm) {
    io.setRPM(rpm);
    Logger.recordOutput("Intake/SetRPM", rpm);
  }

  /** Stops the intake. */
  public void stop() {
    io.stop();
    Logger.recordOutput("Intake/Stopped", true);
  }

  /**
   * Enables or disables brake mode for the intake motors.
   *
   * @param enable True to enable brake mode, false for coast mode.
   */
  public void setBrakeMode(boolean enable) {
    io.setBrakeMode(enable);
    Logger.recordOutput("Intake/BrakeMode", enable);
  }

  /**
   * Returns whether coral is currently detected.
   *
   * @return True if coral is detected, false otherwise.
   */
  public boolean isCoralIn() {
    return !inputs.coralIn;
  }

  public boolean isCoralSet() {
    return !inputs.coralSet && inputs.coralIn;
  }

  /**
   * Returns the current RPM of the intake.
   *
   * @return The RPM of the intake.
   */

  /**
   * Returns the current voltage applied to the intake motors.
   *
   * @return The applied voltage.
   */
  public Command runIntake(double rpm) {
    return this.startEnd(() -> setRPM(rpm), () -> stop());
  }

  public Command runIntakeReverse(double rpm) {
    return this.startEnd(() -> setRPM(-rpm), () -> stop());
  }
}
