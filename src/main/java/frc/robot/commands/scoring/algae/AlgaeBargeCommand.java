package frc.robot.commands.scoring.algae;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorNoPivotCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class AlgaeBargeCommand extends SequentialCommandGroup {
    
    public AlgaeBargeCommand(EndEffectorSubsystem endEffector, ElevatorSubsystem elevator) {
        super(
            new ParallelCommandGroup(
                new PivotCommand(endEffector, 2.746),
                new ElevatorNoPivotCommand(21, elevator))
            // new PivotCommand(endEffector, 0.85)
        );
    }
    
}
