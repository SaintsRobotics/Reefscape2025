// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorDefaultCommand extends Command {
  private final ElevatorSubsystem m_elevatorSubsystem;
  private final DoubleSupplier m_axis;
  private final BooleanSupplier m_manualControllerEnabled;

  private double setpoint;

  /** Creates a new ElevatorDefaultCommand. */
  public ElevatorDefaultCommand(DoubleSupplier axis, BooleanSupplier manualControlEnabled,
      ElevatorSubsystem elevatorSubsystem) {
    addRequirements(elevatorSubsystem);

    m_elevatorSubsystem = elevatorSubsystem;
    m_axis = axis;
    m_manualControllerEnabled = manualControlEnabled;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setpoint = m_elevatorSubsystem.getCurrentHeight();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_manualControllerEnabled.getAsBoolean()) {
      // update setpoint
      setpoint += MathUtil.applyDeadband(m_axis.getAsDouble(), IOConstants.kControllerDeadband)
          * IOConstants.kElevatorAxisScalar;

      // clamp to bounds
      setpoint = MathUtil.clamp(setpoint, ElevatorConstants.kElevatorBottom, ElevatorConstants.kElevatorTop);

      m_elevatorSubsystem.setHeight(setpoint); // interlocks will take care of invalid configurations
    }
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
