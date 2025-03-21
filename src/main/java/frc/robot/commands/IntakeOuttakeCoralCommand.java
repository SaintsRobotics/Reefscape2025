// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem.IntakeState;

public class IntakeOuttakeCoralCommand extends Command {

  EndEffectorSubsystem m_endEffectorSubsystem;
  IntakeState m_intakeState;

  /** Creates a new CoralCommand. */
  public IntakeOuttakeCoralCommand(EndEffectorSubsystem endEffectorSubsystem, IntakeState intakeState) {
    /*
     * We aren't requiring the end effector subsystem here because moving the
     * intake wheels does not affect the safety of the bot in any other way,
     * and this allows us to also simultaneously run commands that pivot the end
     * effector
     */

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
  public void execute() {
  }

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
    } else if (m_intakeState == IntakeState.OuttakeCoral) {
      return !m_endEffectorSubsystem.isHolding();
    }

    return m_intakeState != m_endEffectorSubsystem.getIntakeState();
  }
}
