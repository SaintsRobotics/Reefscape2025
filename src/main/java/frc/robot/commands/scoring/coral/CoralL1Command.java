package frc.robot.commands.scoring.coral;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorNoPivotCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class CoralL1Command extends SequentialCommandGroup {

    public CoralL1Command(EndEffectorSubsystem endEffector, ElevatorSubsystem elevator) {
        addCommands(
            // new ConditionalCommand(
            //     new UndoCoralL4Command(endEffector, elevator),
            //     // new ConditionalCommand(new PivotCommand(endEffector, 1.3), new InstantCommand(), () -> elevator.getCurrentHeight() > 1),
            //     new PivotCommand(endEffector, 1.3),
            //     () -> elevator.getCurrentHeight() >= 12),
            // // new PivotCommand(endEffector, 1.3),
            // new ElevatorCommand(ElevatorConstants.kL1Height, elevator, endEffector),
            // new PivotCommand(endEffector, 0.05)
            new ParallelCommandGroup(new PivotCommand(endEffector, 0.36),
            new ElevatorNoPivotCommand(ElevatorConstants.kL1Height, elevator)),
            new PivotCommand(endEffector, 0.05)
        );
    }
    
}
