// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map.Entry;

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

    final Entry<Double, Pair<Double, Double>> currentLimit = EndEffectorConstants.kSafePivotPositions.floorEntry(currentPosition);
    final Entry<Double, Pair<Double, Double>> higherLimit = EndEffectorConstants.kSafePivotPositions.higherEntry(currentPosition);
    final Entry<Double, Pair<Double, Double>> lowerLimit = EndEffectorConstants.kSafePivotPositions.lowerEntry(currentLimit.getKey());

    final Pair<Double, Double> pivotLimits;

    // check if moving to next pivot limit

    // check if greater than or equal to minimum elevator height for next limit
    if (higherLimit != null && m_desiredHeight >= higherLimit.getKey()) { // going up
      pivotLimits = higherLimit.getValue();
    }
    
    // check is less than the minimum elevator height for current limit
    else if (lowerLimit != null && m_desiredHeight < currentLimit.getKey()) { // going down
      pivotLimits = lowerLimit.getValue();
    }
    
    else { // staying at level
      pivotLimits = currentLimit.getValue();
    }

    final double pivotPosition = MathUtil.clamp(m_endEffectorSubsystem.getSetpoint(), pivotLimits.getFirst(),
        pivotLimits.getSecond());
    m_endEffectorSubsystem.pivotTo(pivotPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevatorSubsystem.atSetpoint();
  }
}
