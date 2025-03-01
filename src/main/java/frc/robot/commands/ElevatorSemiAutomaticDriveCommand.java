// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map.Entry;
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

  /** Creates a new ElevatorSemiAutomaticDriveCommand. */
  public ElevatorSemiAutomaticDriveCommand(DoubleSupplier speed, EndEffectorSubsystem endEffector, ElevatorSubsystem elevator) {
    addRequirements(endEffector, elevator);

    m_speed = speed;
    m_endEffector = endEffector;
    m_elevator = elevator;
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
    final double speed = m_speed.getAsDouble();

    double pivotSetpoint = m_endEffector.getSetpoint();
    final double currentPosition = m_elevator.getCurrentHeight();

    final Entry<Double, Pair<Double, Double>> currentLimit = EndEffectorConstants.kSafePivotPositions
        .floorEntry(currentPosition);
    final Entry<Double, Pair<Double, Double>> higherLimit = EndEffectorConstants.kSafePivotPositions
        .higherEntry(currentPosition);
    final Entry<Double, Pair<Double, Double>> lowerLimit = EndEffectorConstants.kSafePivotPositions
        .lowerEntry(currentLimit.getKey());

    // clamp to current
    pivotSetpoint = MathUtil.clamp(pivotSetpoint, currentLimit.getValue().getFirst(),
        currentLimit.getValue().getSecond());

    // check direction
    if (speed > 0) { // going up
      pivotSetpoint = MathUtil.clamp(pivotSetpoint, higherLimit.getValue().getFirst(),
          higherLimit.getValue().getSecond());
    } else { // going down
      pivotSetpoint = MathUtil.clamp(pivotSetpoint, lowerLimit.getValue().getFirst(),
          lowerLimit.getValue().getSecond());
    }

    m_endEffector.pivotTo(pivotSetpoint);
    m_elevator.setSpeed(speed);
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
