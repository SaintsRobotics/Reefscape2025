// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Robot;
import frc.robot.subsystems.EndEffectorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceGrabCoralCommand extends SequentialCommandGroup {
  /** Creates a new PlaceGrabCoralCommand. */
  public PlaceGrabCoralCommand(EndEffectorSubsystem endEffectorSubsystem, boolean outtake) {
    if (outtake) { //TODO: tune constants, and make dynamic based on elevator height?
      addCommands(
        new ParallelDeadlineGroup(new Command() {
          @Override
          public boolean isFinished() {
              return !endEffectorSubsystem.isHolding() || Robot.isSimulation();
          }
        }, new StartEndCommand(endEffectorSubsystem::intakeCoral, endEffectorSubsystem::stopEffector))
        //new PivotCommand(endEffectorSubsystem, 0)
        );
    }
    else {
      addCommands(
        //new PivotCommand(endEffectorSubsystem, 0),
        new ParallelDeadlineGroup(new Command() {
          @Override
          public boolean isFinished() {
              return endEffectorSubsystem.isHolding() || Robot.isSimulation();
          }
        }, new StartEndCommand(endEffectorSubsystem::intakeCoral, endEffectorSubsystem::stopEffector))
        //new PivotCommand(endEffectorSubsystem, 0)
        );
    }
  }
}
