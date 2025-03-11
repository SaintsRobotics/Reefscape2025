package frc.robot.commands.scoring.algae;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class AlgaeL4Command extends SequentialCommandGroup {

    public AlgaeL4Command(EndEffectorSubsystem endEffector, ElevatorSubsystem elevator) {
        addCommands();
    }
    
}
