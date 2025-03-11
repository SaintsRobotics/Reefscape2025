package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;

public class SimpleDriveForwards extends SequentialCommandGroup {

    public SimpleDriveForwards(DriveSubsystem driveSubsystem, double time, double speed) {
        addCommands(
            new ParallelDeadlineGroup(new WaitCommand(time), new RunCommand(() -> driveSubsystem.drive(speed, 0, 0, false), driveSubsystem)),
            new ParallelDeadlineGroup(new WaitCommand(0.1), new RunCommand(() -> driveSubsystem.drive(0, 0, 0, false), driveSubsystem))
        );
    }

}
