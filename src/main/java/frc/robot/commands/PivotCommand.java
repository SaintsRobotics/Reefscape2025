// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotCommand extends Command {
  private final EndEffectorSubsystem m_endEffector;
  private final double m_setpoint;

  /** Creates a new PivotCommand. */
  public PivotCommand(EndEffectorSubsystem endEffector, double setpoint) {
    addRequirements(endEffector);

    m_endEffector = endEffector;
    m_setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_endEffector.pivotTo(m_setpoint); // needs to be in execute in case smart pivot changes setpoint
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_endEffector.getSetpoint() == m_setpoint && m_endEffector.atSetpoint();
  }
}
