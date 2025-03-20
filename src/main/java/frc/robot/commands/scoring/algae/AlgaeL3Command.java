package frc.robot.commands.scoring.algae;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorNoPivotCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class AlgaeL3Command extends SequentialCommandGroup {

    public AlgaeL3Command(EndEffectorSubsystem endEffector, ElevatorSubsystem elevator) {
        super(
            new ParallelCommandGroup(
                new PivotCommand(endEffector, 2.672),
                new ElevatorNoPivotCommand(11.2, elevator))
        );
    }
    
}
