package frc.robot.commands.scoring.algae;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.ElevatorNoPivotCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class AlgaeProcessorCommand extends SequentialCommandGroup {

    public AlgaeProcessorCommand(EndEffectorSubsystem endEffector, ElevatorSubsystem elevator) {
        super(
            new ParallelCommandGroup(new PivotCommand(endEffector, 2.727),
            new ElevatorNoPivotCommand(ElevatorConstants.kL1Height, elevator))
        );
    }
    
}
