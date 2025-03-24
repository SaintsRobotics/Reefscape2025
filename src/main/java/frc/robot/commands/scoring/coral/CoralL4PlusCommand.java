package frc.robot.commands.scoring.coral;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.ElevatorNoPivotCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class CoralL4PlusCommand extends SequentialCommandGroup {

    public CoralL4PlusCommand(EndEffectorSubsystem endEffector, ElevatorSubsystem elevator) {
        super(
            new ParallelCommandGroup(
                new PivotCommand(endEffector, 0.55), 
                new ElevatorNoPivotCommand(ElevatorConstants.kL4Height + 0.75, elevator))
        );
    }
    
}