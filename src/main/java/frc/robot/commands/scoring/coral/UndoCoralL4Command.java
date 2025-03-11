package frc.robot.commands.scoring.coral;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class UndoCoralL4Command extends SequentialCommandGroup {
    
    public UndoCoralL4Command(EndEffectorSubsystem endEffector, ElevatorSubsystem elevator) {
        addCommands(
            new PivotCommand(endEffector, 0.698),
            new ElevatorCommand(13.8, elevator, endEffector),
            new PivotCommand(endEffector, 1.608),

            new ElevatorCommand(ElevatorConstants.kL3Height, elevator, endEffector)
        );
    }

}
