// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorNoPivotCommand extends Command {
  private final ElevatorSubsystem m_elevatorSubsystem;

  private final double m_desiredHeight;

  /** Creates a new ElevatorPivotCommand. */
  public ElevatorNoPivotCommand(double desiredHeight, ElevatorSubsystem elevatorSubsystem) {
    addRequirements(elevatorSubsystem);

    m_desiredHeight = desiredHeight;
    m_elevatorSubsystem = elevatorSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevatorSubsystem.setHeight(m_desiredHeight);
    // m_endEffectorSubsystem.pivotTo(m_desiredPivot);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.setHeight(m_elevatorSubsystem.getCurrentHeight());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevatorSubsystem.getHeightSetpoint() == m_desiredHeight && m_elevatorSubsystem.atSetpoint();
  }
}
