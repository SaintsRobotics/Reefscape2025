// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.EndEffectorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceGrabAlgaeCommand extends SequentialCommandGroup {
  /** Creates a new PlaceGrabAlgaeCommand. */
  public PlaceGrabAlgaeCommand(EndEffectorSubsystem endEffectorSubsystem) {
    if (endEffectorSubsystem.isHolding()) { //TODO: tune constants, and make dynamic based on elevator height?
      addCommands(
        new PivotCommand(endEffectorSubsystem, 0),
        new TimedCommand(() -> endEffectorSubsystem.outtakeAlgae(), 0),
        new PivotCommand(endEffectorSubsystem, 0)
        );
    }
    else {
      addCommands(
        new PivotCommand(endEffectorSubsystem, 0),
        new TimedCommand(() -> endEffectorSubsystem.intakeAlgae(), 0),
        new PivotCommand(endEffectorSubsystem, 0)
        );
    }
  }
}
