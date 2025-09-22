// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.awt.Color;

/**
 * Implementation of the LedIO interface using the CANdle LED controller. This class configures and
 * controls the LED strip connected to the CANdle.
 */
public class Led extends SubsystemBase {
  /** Number of LEDs in the strip. */
  private final int ledCount = 200;

  /** CANdle LED controller instance. */
  private final CANdle candle;

  /** Buffered animation for the LED strip. */
  private Animation bufferedAnimation = new RainbowAnimation(0.7, 0.2, ledCount);

  /** Constructor to initialize and configure the CANdle LED controller. */
  public Led(int candleID) {
    candle = new CANdle(candleID, "rio");

    CANdleConfiguration candleConfiguration = new CANdleConfiguration();
    candleConfiguration.statusLedOffWhenActive = true;
    candleConfiguration.disableWhenLOS = false;
    candleConfiguration.stripType = LEDStripType.RGB;
    candleConfiguration.brightnessScalar = 1.0;
    candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(candleConfiguration, 100);
    candle.clearAnimation(0);
  }

  /**
   * Sets the LED strip to a solid color.
   *
   * @param color The color to set the LEDs to.
   */
  public void strobeAnimation(Color color) {
    candle.clearAnimation(0);
    bufferedAnimation =
        new StrobeAnimation(color.getRed(), color.getGreen(), color.getBlue(), 0, 1, ledCount);
    candle.animate(bufferedAnimation);
  }
/**
   * Sets the LED strip to a solid color .
   *
   * @param color The color to set the LEDs to.
   * @see strobeAnimation
   */
  public Command setColor(Color color) {
    return new Command() {
      @Override
      public void initialize() {
        strobeAnimation(color);
      }
/** sets the leds to black, regardless of the argument */
      @Override
      public void end(boolean interrupted) {
        strobeAnimation(Color.BLACK);
      }
    };
  }
}
