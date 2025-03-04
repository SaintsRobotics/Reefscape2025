// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LEDBufferSet extends Command {
  public static enum LED_STATES {
    ELEVATE_PURPLE,
    DESCEND_YELLOW,
    INTAKE_GREEN,
    OUTTAKE_WHITE,
    RED_TEAM_RED,
    BLUE_TEAM_BLUE,
    FINAL_RAINBOW,
  }
  private LED m_leds;
  private LED_STATES m_states;
  /** Creates a new LEDBufferSet. */
  public LEDBufferSet(LED_STATES states, LED LED) {
    // Use addRequirements() here to declare subsystem dependencies.
    states = m_states;
    LED = m_leds;
    addRequirements(m_leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (m_states) {
      case ELEVATE_PURPLE:
        m_leds.setBufferColor(Color.kDarkViolet);
        break;
      case DESCEND_YELLOW:
        m_leds.setBufferColor(Color.kGoldenrod);
        break;
      case INTAKE_GREEN:
        m_leds.setBufferColor(Color.kForestGreen);
        break;
      case OUTTAKE_WHITE:
        m_leds.setBufferColor(Color.kDarkViolet);
        break;
      case BLUE_TEAM_BLUE:
        m_leds.setBufferColor(Color.kBlue);
        break;
      case RED_TEAM_RED:
        m_leds.setBufferColor(Color.kRed);
        break;
      case FINAL_RAINBOW:
        m_leds.setBufferPattern(LEDPattern.rainbow(255, 128));
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
