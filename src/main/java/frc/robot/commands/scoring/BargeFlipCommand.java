// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BargeFlipCommand extends Command {

  private final EndEffectorSubsystem m_endEffector;

  private final Timer m_timer = new Timer();

  /** Creates a new BargeFlipCommand. */
  public BargeFlipCommand(EndEffectorSubsystem endEffector) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(endEffector);

    m_endEffector = endEffector;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();

    m_endEffector.pivotTo(0.36);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.get() > 0.15) {
      m_endEffector.outtakeAlgae();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_endEffector.stopEffector();
    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_endEffector.atSetpoint() && m_endEffector.getSetpoint() == 0.36;
  }
}
