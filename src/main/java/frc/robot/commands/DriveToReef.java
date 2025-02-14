// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.FindNearest;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToReef extends Command {

  private final DriveSubsystem m_driveSubsystem;
  private Pose2d m_targetPose = new Pose2d();

  private Command m_driveToPose;

  /** Creates a new DriveToReef. */
  public DriveToReef(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_driveSubsystem = driveSubsystem;

    // Safety net in case the pose is not set
    m_targetPose = m_driveSubsystem.getPose();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_targetPose = FindNearest.getNearestScoringLocation(m_driveSubsystem.getPose());

    m_driveToPose = new DriveToPose(m_driveSubsystem, m_targetPose);

    m_driveToPose.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !DriveConstants.kAutoDriving || m_driveToPose.isFinished();
  }
}
