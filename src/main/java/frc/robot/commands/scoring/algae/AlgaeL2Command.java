package frc.robot.commands.scoring.algae;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.commands.scoring.coral.UndoCoralL4Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class AlgaeL2Command extends SequentialCommandGroup {

    public AlgaeL2Command(EndEffectorSubsystem endEffector, ElevatorSubsystem elevator) {
        addCommands(
            new ConditionalCommand(
                new UndoCoralL4Command(endEffector, elevator),
                new InstantCommand(),
                () -> elevator.getCurrentHeight() >= 12),
            new PivotCommand(endEffector, .3 * 2 * Math.PI),
            new ElevatorCommand(6.1, elevator, endEffector), 
            new PivotCommand(endEffector, 2.746)
        );
    }
    
}
