// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberCommand extends Command {
  
  private final ClimberSubsystem m_climber;

  private final double m_setpoint;

  /** Creates a new setWinding. */
  public ClimberCommand(ClimberSubsystem climberSubsystem, double targetSetpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climberSubsystem);

    m_climber = climberSubsystem;
    m_setpoint = targetSetpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.setLockPosition(ClimberConstants.kUnlockedPosition);
    m_climber.setWindingSetpoint(m_setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.setLockPosition(ClimberConstants.kLockedPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climber.windingAtSetpoint();
  }
}
