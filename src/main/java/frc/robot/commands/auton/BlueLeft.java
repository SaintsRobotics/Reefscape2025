package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CoralCommand;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.scoring.L1Command;
import frc.robot.commands.scoring.L4Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem.IntakeState;
import frc.robot.utils.FindNearest;

public class BlueLeft extends SequentialCommandGroup {

    public BlueLeft(DriveSubsystem driveSubsystem, ElevatorSubsystem elevator, EndEffectorSubsystem endEffector) {
        Pose2d startingPose = new Pose2d(7.745, 7.525, new Rotation2d(Math.toRadians(180)));
        addCommands(
            // new ParallelDeadlineGroup(new WaitCommand(time), new RunCommand(() -> driveSubsystem.drive(speed, 0, 0, false), driveSubsystem)),
            // new ParallelDeadlineGroup(new WaitCommand(0.1), new RunCommand(() -> driveSubsystem.drive(0, 0, 0, false), driveSubsystem)),
            // new ParallelDeadlineGroup(new WaitCommand(3), new InstantCommand(() -> endEffector.outtakeCoral(), endEffector)),
            // new InstantCommand((() -> endEffector.stopEffector()), endEffector)
            // new InstantCommand(() -> driveSubsystem.setHeading(Math.toRadians(180)), driveSubsystem),
            new InstantCommand(() -> driveSubsystem.resetOdometry(startingPose), driveSubsystem),
            new WaitCommand(1),
            new DriveToPose(driveSubsystem, FindNearest.blueScoringLocations[9].plus(new Transform2d(FindNearest.blueScoringLocations[9], startingPose).times(0.15))),
            new L4Command(endEffector, elevator, () -> true),
            new DriveToPose(driveSubsystem, FindNearest.blueScoringLocations[9]),
            new ParallelDeadlineGroup(new WaitCommand(1), new InstantCommand(() -> endEffector.outtakeCoral(), endEffector)),
            new InstantCommand((() -> endEffector.stopEffector()), endEffector),
            new DriveToPose(driveSubsystem, FindNearest.blueScoringLocations[9].plus(new Transform2d(startingPose, FindNearest.blueScoringLocations[9]).times(-0.15))),
            new L1Command(endEffector, elevator, () -> true),
            new DriveToPose(driveSubsystem, FindNearest.blueSources[1]),
            new CoralCommand(endEffector, IntakeState.OuttakeCoral),
            new DriveToPose(driveSubsystem, FindNearest.blueScoringLocations[7].plus(new Transform2d(FindNearest.blueSources[1], FindNearest.blueScoringLocations[7]).times(-0.15))),
            new L4Command(endEffector, elevator, () -> true),
            new DriveToPose(driveSubsystem, FindNearest.blueScoringLocations[7]),
            new ParallelDeadlineGroup(new WaitCommand(1), new InstantCommand(() -> endEffector.outtakeCoral(), endEffector)),
            new InstantCommand((() -> endEffector.stopEffector()), endEffector),
            new DriveToPose(driveSubsystem, FindNearest.blueScoringLocations[7].plus(new Transform2d(FindNearest.blueSources[1], FindNearest.blueScoringLocations[7]).times(-0.15)))    
        );
    }

}
