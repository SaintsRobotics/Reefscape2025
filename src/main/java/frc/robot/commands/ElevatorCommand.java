// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommand extends Command {
  private final ElevatorSubsystem m_elevatorSubsystem;
  private final EndEffectorSubsystem m_endEffectorSubsystem;

  private final double m_desiredHeight;

  /** Creates a new ElevatorCommand. */
  public ElevatorCommand(double desiredHeight, ElevatorSubsystem elevatorSubsystem,
      EndEffectorSubsystem endEffectorSubsystem) {
    addRequirements(elevatorSubsystem, endEffectorSubsystem);

    m_desiredHeight = desiredHeight;
    m_elevatorSubsystem = elevatorSubsystem;
    m_endEffectorSubsystem = endEffectorSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevatorSubsystem.setHeight(m_desiredHeight);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double currentPosition = m_elevatorSubsystem.getCurrentHeight();

    Pair<Double, Double> pivotLimits;

    // check direction
    if (currentPosition < m_desiredHeight) { // going up
      pivotLimits = EndEffectorConstants.kSafePivotPositions.higherEntry(currentPosition).getValue();
    } else { // going down
      pivotLimits = EndEffectorConstants.kSafePivotPositions.lowerEntry(currentPosition).getValue();
    }

    final double pivotPosition = MathUtil.clamp(m_endEffectorSubsystem.getSetpoint(), pivotLimits.getFirst(),
        pivotLimits.getSecond());
    m_endEffectorSubsystem.pivotTo(pivotPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.setHeight(m_elevatorSubsystem.getCurrentHeight()); // TODO: replace with default command that
                                                                           // stops elevator (unless manually driven)
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevatorSubsystem.atSetpoint();
  }
}
