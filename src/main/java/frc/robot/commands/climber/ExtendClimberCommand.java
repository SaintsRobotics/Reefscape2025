// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExtendClimberCommand extends SequentialCommandGroup {
  /** Creates a new extendClimber. */
  public ExtendClimberCommand(ClimberSubsystem climberSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Retract the winch slightly
      new SetWindingCommand(climberSubsystem, Constants.ClimberConstants.kWindingUnlockPosition),
      // Release the servo lock
      new InstantCommand(() -> climberSubsystem.setLockPosition(Constants.ClimberConstants.kUnlockedPosition)),
      // Unwind the winch
      new SetWindingCommand(climberSubsystem, Constants.ClimberConstants.kWindingExtendedPosition),
      // Engage the servo lock
      new InstantCommand(() -> climberSubsystem.setLockPosition(Constants.ClimberConstants.kLockedPosition)));
  }
}
