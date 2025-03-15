// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HapticCommand extends Command {
  private XboxController m_controller;
  private double m_amount;
  private double m_time;
  private Timer m_timer = new Timer();
  /** Creates a new UpdateHaptics. */
  public HapticCommand(XboxController controller, double time, double amount) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_controller = controller;
    m_amount = amount;
    m_time = time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("haptic",true);
    m_timer.reset();
    m_controller.setRumble(GenericHID.RumbleType.kBothRumble, m_amount);
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_controller.setRumble(GenericHID.RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return m_timer.get()>m_time;
  }
}
