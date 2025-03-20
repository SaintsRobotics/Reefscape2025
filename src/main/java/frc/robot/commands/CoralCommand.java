// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem.IntakeState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralCommand extends Command {

  EndEffectorSubsystem m_endEffectorSubsystem;
  IntakeState m_intakeState;

  /** Creates a new CoralCommand. */
  public CoralCommand(EndEffectorSubsystem endEffectorSubsystem, IntakeState intakeState) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_endEffectorSubsystem = endEffectorSubsystem;
    m_intakeState = intakeState;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_endEffectorSubsystem.setIntakeState(m_intakeState);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_endEffectorSubsystem.setIntakeState(IntakeState.Idle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (Robot.isSimulation()) {
      return true;
    }

    if (m_intakeState == IntakeState.IntakeCoral) {
      return m_endEffectorSubsystem.isHolding();
    } 
    else if (m_intakeState == IntakeState.OuttakeCoral) {
      return !m_endEffectorSubsystem.isHolding();
    }

    return m_intakeState != m_endEffectorSubsystem.getIntakeState();
  }
}
