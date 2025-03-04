// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  private AddressableLED m_LED = new AddressableLED(0);
  private AddressableLEDBuffer m_Buffer = new AddressableLEDBuffer(0);
  /** Creates a new LED. */
  public LED() {
    m_LED.setLength(0);
    m_LED.setData(m_Buffer);
    m_LED.start();
  }
  public void setBufferColor(Color color) {
    Color setColor = color;
    m_Buffer.setLED(0, setColor);
  }
  public void setBufferPattern(LEDPattern pattern) {
    LEDPattern setPattern = pattern;
    setPattern.applyTo(m_Buffer);
  }
  @Override
  public void periodic() {
    m_LED.setData(m_Buffer);
  }
}
