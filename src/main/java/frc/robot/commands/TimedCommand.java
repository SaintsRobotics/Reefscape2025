// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TimedCommand extends Command {
  private final Timer m_timer;
  private final Command m_command;
  private final double m_timeout;

  /**
   * 
   * @param command The command to run
   * @param time Timeout in seconds
   */
  public TimedCommand(Command command, double timeout) {
    m_timer = new Timer();
    m_command = command;
    m_timeout = timeout;
  }

  /**
   * 
   * @param runnable Runnable to continously run
   * @param timeout Timeout in seconds
   * @param subsystems subsystems to require
   */
  public TimedCommand(Runnable runnable, double timeout, Subsystem... subsystems) {
    this(new RunCommand(runnable, subsystems), timeout);
  }

  @Override
  public void initialize() {
    m_timer.restart();
    m_command.schedule();
  }

  @Override
  public void end(boolean interrupted) {
      m_command.cancel();
  }

  @Override
  public boolean isFinished() {
      return m_command.isFinished() || m_timer.hasElapsed(m_timeout);
  }
}
