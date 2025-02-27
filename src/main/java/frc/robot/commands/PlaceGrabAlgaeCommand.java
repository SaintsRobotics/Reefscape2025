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
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // TODO: detect if placing or grabbing
    
    // TODO: create a command sequence

    // TODO: ensure each command has an end interrupted handler to return to original state
  }
}
