// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.utils.Interlocks;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceGrabAlgaeCommand extends SequentialCommandGroup {
  /** Creates a new PlaceGrabAlgaeCommand. */
  /**
   * 
   * @param endEffectorSubsystem
   * @param outtake True if outtaking. False if intaking
   */
  public PlaceGrabAlgaeCommand(EndEffectorSubsystem endEffectorSubsystem, boolean outtake, Interlocks interlocks) {
    if (outtake) { //TODO: tune constants, and make dynamic based on elevator height?
      addCommands(
        //new PivotCommand(endEffectorSubsystem, 0),
        new ParallelDeadlineGroup(new WaitCommand(0), new StartEndCommand(endEffectorSubsystem::outtakeAlgae, endEffectorSubsystem::stopEffector)),
        new InstantCommand(() -> interlocks.setAlgeaHolding(false))
        //new PivotCommand(endEffectorSubsystem, 0)
        );
    }
    else {
      interlocks.setAlgeaHolding(true);
      addCommands(
        //new PivotCommand(endEffectorSubsystem, 0),
        new ParallelDeadlineGroup(new WaitCommand(0), new StartEndCommand(endEffectorSubsystem::intakeAlgae, endEffectorSubsystem::stopEffector)),
        new InstantCommand(() -> interlocks.setAlgeaHolding(true))
        //new PivotCommand(endEffectorSubsystem, 0)
        );
      
    }
  }
}
