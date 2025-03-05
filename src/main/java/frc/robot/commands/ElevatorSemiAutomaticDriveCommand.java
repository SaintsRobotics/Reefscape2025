// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.Map.Entry;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorSemiAutomaticDriveCommand extends Command {
  private final DoubleSupplier m_speed;
  private final EndEffectorSubsystem m_endEffector;
  private final ElevatorSubsystem m_elevator;

  // used to prevent boundary lockups
  private final BooleanSupplier m_lockupHint;

  /** Creates a new ElevatorSemiAutomaticDriveCommand. */
  public ElevatorSemiAutomaticDriveCommand(DoubleSupplier speed,  BooleanSupplier lockupHint, EndEffectorSubsystem endEffector, ElevatorSubsystem elevator) {
    addRequirements(endEffector, elevator);

    m_speed = speed;
    m_endEffector = endEffector;
    m_elevator = elevator;
    m_lockupHint = lockupHint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // because we cant do position prediction here, we need to use more restrictive
    // pivot adjustments
    // always clamp using current, and also clamp to next
    final double currentPosition = m_elevator.getCurrentHeight();

    final Entry<Double, List<Pair<Double, Double>>> currentLimit = EndEffectorConstants.kSafePivotPositions.floorEntry(currentPosition);
    final Entry<Double, List<Pair<Double, Double>>> higherLimit = EndEffectorConstants.kSafePivotPositions.higherEntry(currentLimit.getKey());
    final Entry<Double, List<Pair<Double, Double>>> lowerLimit = EndEffectorConstants.kSafePivotPositions.lowerEntry(currentLimit.getKey());

    final List<Pair<Double, Double>> pivotLimits;

    // check if moving to next pivot limit

    // check if greater than or equal to minimum elevator height for next limit
    if (m_speed.getAsDouble() > 0) { // going up
      pivotLimits = higherLimit.getValue();
    }
    
    // check is less than the minimum elevator height for current limit
    else if (m_speed.getAsDouble() < 0) { // going down
      pivotLimits = lowerLimit.getValue();
    }
    
    else { // staying at level
      pivotLimits = currentLimit.getValue();
    }


    boolean needsClamp = true;
    double pivotPosition = m_endEffector.getSetpoint();

    // detect future lockup
    if (m_lockupHint.getAsBoolean()) {
      for (Pair<Double, Double> limit : pivotLimits) {
        if (pivotPosition >= limit.getFirst() && pivotPosition <= limit.getSecond()) {
          needsClamp = false;
        }
      }
  
      if (needsClamp) {
        pivotPosition = MathUtil.clamp(pivotPosition, pivotLimits.get(0).getFirst(),
        pivotLimits.get(0).getFirst());
      }
    }
    
    needsClamp = true;
    for (Pair<Double, Double> limit : currentLimit.getValue()) {
      if (pivotPosition >= limit.getFirst() && pivotPosition <= limit.getSecond()) {
        needsClamp = false;
        break;
      }
    }

    if (needsClamp) {
      pivotPosition = MathUtil.clamp(pivotPosition, currentLimit.getValue().get(0).getFirst(),
        currentLimit.getValue().get(0).getFirst());
    }

    m_endEffector.pivotTo(pivotPosition, true);
    m_elevator.setSpeed(m_speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
