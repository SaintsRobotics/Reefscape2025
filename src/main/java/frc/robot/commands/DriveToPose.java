// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToPose extends Command {

  private final DriveSubsystem m_driveSubsystem;
  private Pose2d currentPose;
  private final Pose2d m_targetPose;
  private Timer m_timer = new Timer();

  // TODO: Move to Constants AND DO NOT TEST THIS ON THE GROUND
  private final PIDController xController = new PIDController(8.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(8.0, 0.0, 0.0);
  private final PIDController thetaController = new PIDController(8.0, 0.0, 0.0);

  /** Creates a new DriveToPose. */
  public DriveToPose(DriveSubsystem driveSubsystem, Pose2d targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);

    thetaController.enableContinuousInput(0, Math.PI * 2);

    m_driveSubsystem = driveSubsystem;
    m_targetPose = targetPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();

    xController.reset();
    yController.reset();
    thetaController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPose = m_driveSubsystem.getPose();

    double xSpeed = xController.calculate(currentPose.getX(), m_targetPose.getX());
    double ySpeed = yController.calculate(currentPose.getY(), m_targetPose.getY());
    double thetaSpeed = thetaController.calculate(currentPose.getRotation().getRadians(),
        m_targetPose.getRotation().getRadians());

    if (DriveConstants.kAutoDriving) m_driveSubsystem.drive(xSpeed, ySpeed, thetaSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    currentPose = m_driveSubsystem.getPose();
    return !DriveConstants.kAutoDriving || (m_timer.get() > 5) || (currentPose.getTranslation().getDistance(m_targetPose.getTranslation()) <= 0.05) // meters, add to constants later
      && Math.abs(currentPose.getRotation().minus(m_targetPose.getRotation()).getRadians()) <= Math.toRadians(1.5); // 1.5 degrees, add to constants later
  }
}
