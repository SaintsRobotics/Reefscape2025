package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.PlaceGrabCoralCommand;
import frc.robot.commands.scoring.L1Command;
import frc.robot.commands.scoring.L4Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.FindNearest;

public class RedRight extends SequentialCommandGroup {

    public RedRight(DriveSubsystem driveSubsystem, ElevatorSubsystem elevator, EndEffectorSubsystem endEffector) {
        Pose2d startingPose = AllianceFlipUtil.apply(new Pose2d(7.745, 7.525, new Rotation2d(Math.toRadians(0))));
        addCommands(
            // new ParallelDeadlineGroup(new WaitCommand(time), new RunCommand(() -> driveSubsystem.drive(speed, 0, 0, false), driveSubsystem)),
            // new ParallelDeadlineGroup(new WaitCommand(0.1), new RunCommand(() -> driveSubsystem.drive(0, 0, 0, false), driveSubsystem)),
            // new ParallelDeadlineGroup(new WaitCommand(3), new InstantCommand(() -> endEffector.outtakeCoral(), endEffector)),
            // new InstantCommand((() -> endEffector.stopEffector()), endEffector)
            new InstantCommand(() -> driveSubsystem.setHeading(Math.toRadians(0)), driveSubsystem),
            new InstantCommand(() -> driveSubsystem.resetOdometry(startingPose), driveSubsystem),
            new WaitCommand(1),
            new DriveToPose(driveSubsystem, FindNearest.redScoringLocations[9].plus(new Transform2d(FindNearest.redScoringLocations[9], startingPose).times(0.15))),
            // new L4Command(endEffector, elevator, () -> true),
            new DriveToPose(driveSubsystem, FindNearest.redScoringLocations[9]),
            // new ParallelDeadlineGroup(new WaitCommand(1), new InstantCommand(() -> endEffector.outtakeCoral(), endEffector)),
            // new InstantCommand((() -> endEffector.stopEffector()), endEffector),
            new DriveToPose(driveSubsystem, FindNearest.redScoringLocations[9].plus(new Transform2d(startingPose, FindNearest.redScoringLocations[9]).times(0.15))),
            // new L1Command(endEffector, elevator, () -> true),
            new DriveToPose(driveSubsystem, FindNearest.redSources[1]),
            // new PlaceGrabCoralCommand(endEffector, false)
            new DriveToPose(driveSubsystem, FindNearest.redScoringLocations[7].plus(new Transform2d(FindNearest.redSources[1], FindNearest.redScoringLocations[7]).times(-0.15))),
            // new L4Command(endEffector, elevator, () -> true),
            new DriveToPose(driveSubsystem, FindNearest.redScoringLocations[7]),
            // new ParallelDeadlineGroup(new WaitCommand(1), new InstantCommand(() -> endEffector.outtakeCoral(), endEffector)),
            // new InstantCommand((() -> endEffector.stopEffector()), endEffector),
            new DriveToPose(driveSubsystem, FindNearest.redScoringLocations[7].plus(new Transform2d(FindNearest.redSources[1], FindNearest.redScoringLocations[7]).times(-0.15)))    
        );
    }

}
