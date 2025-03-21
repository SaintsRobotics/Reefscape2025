// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

  private static final int kPort = 9;
  private static final int kLength = 60;

  private final AddressableLED m_led = new AddressableLED(kPort);
  private final AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(kLength);
  private final AddressableLEDBufferView[] sections = { 
    m_buffer.createView(0, kLength / 2), 
    m_buffer.createView(kLength / 2, kLength - 1) 
  };

  // all hues at maximum saturation and half brightness
  private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);

  // Our LED strip has a density of 120 LEDs per meter
  private static final Distance kLedSpacing = Meters.of(1 / 120.0);

  // Create a new pattern that scrolls the rainbow pattern across the LED strip,
  // moving at a speed
  // of 1 meter per second.
  private final LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.1), kLedSpacing);

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    m_led.setLength(kLength);
    m_led.start();

    // Set the default command to turn the strip off, otherwise the last colors
    // written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!
    setDefaultCommand(runPatternCommand(m_scrollingRainbow, "Rainbow"));
  }

  @Override
  public void periodic() {
    // Update the buffer with the rainbow animation
    if (DriverStation.isDisabled()) {
      m_scrollingRainbow.applyTo(m_buffer);
    }

    // Periodically send the latest LED color data to the LED strip for it to
    // display
    m_led.setData(m_buffer);

    try {
      SmartDashboard.putString("LED Command", getCurrentCommand().getName());
    } catch(Exception e) {
      SmartDashboard.putString("LED Command", "no cmd");
    }

    // SmartDashboard.putString("LED Command", getCurrentCommand().getName());
  }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command runPatternCommand(LEDPattern pattern, String name) {
    return run(() -> pattern.applyTo(m_buffer)).withName(name);
    // return animate(pattern, " pattern command");
  }

  // public Command animate(LEDPattern animation, String name) {
    // return run(() -> {
    //       updateLEDs(animation);
    //     })
    //     .withName("Animate " + name);
  // }

  private void updateLEDs(LEDPattern animation) {
    for (AddressableLEDBufferView section : sections) {
      animation.applyTo(section);
      for (int i = 0; i < section.getLength(); ++i) {
        // m_buffer.setLED(i, null);(section.getRed(i), section.getGreen(i), section.getBlue(i), 0, i, 1);
        m_buffer.setLED(i, new Color(section.getRed(i), section.getGreen(i), section.getBlue(i)));
      }
    }
  }

  public Command colorSet(int r, int g, int b, String name) {
    SmartDashboard.putString("led", "ya");
    return runPatternCommand(LEDPattern.solid(new Color(r, g, b)), "color: " + name);
  }

  public Command tripleBlink(int r, int g, int b, String name) {
    return runPatternCommand(LEDPattern.solid(new Color(r, g, b)).blink(Seconds.of(1.0 / 6.0)), name)
        .withTimeout(Seconds.of(1.0))
        .withName("Blink: " + name);
  }

  public Command blink(int r, int g, int b, String name) {
    return runPatternCommand(LEDPattern.solid(new Color(r, g, b)).blink(Seconds.of(1.0 / 6.0)), name)
        .withName("Blink: " + name);
  }

  public Command coralModeCommand(BooleanSupplier coralMode, BooleanSupplier isHolding) {
    return new ConditionalCommand(
      new ConditionalCommand(
        runPatternCommand(LEDPattern.solid(new Color(128, 0, 255)), "coral mode"),
        runPatternCommand(LEDPattern.solid(new Color(0, 255, 255)), "intookened coral"),
        isHolding
      ),
      runPatternCommand(LEDPattern.solid(new Color(0, 255, 0)), "algae"),
      coralMode);

    // System.out.println(coralMode.getAsBoolean());
    // if (coralMode.getAsBoolean()) {
    //   if (isHolding.getAsBoolean()) {
    //     return runPatternCommand(LEDPattern.solid(new Color(0, 255, 255)), "intookened coral");
    //   } else {
    //     return runPatternCommand(LEDPattern.solid(new Color(128, 0, 255)), "coral mode");
    //   }
    // } else {
    //   return runPatternCommand(LEDPattern.solid(new Color(0, 255, 0)), "algae");
    // }
  }
}
