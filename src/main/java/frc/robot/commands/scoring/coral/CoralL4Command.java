package frc.robot.commands.scoring.coral;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class CoralL4Command extends SequentialCommandGroup {

    public CoralL4Command(EndEffectorSubsystem endEffector, ElevatorSubsystem elevator) {
        addCommands(
            new PivotCommand(endEffector, 1.3),
            // new ElevatorCommand(ElevatorConstants.kL3Height, elevator, endEffector),
            // new PivotCommand(endEffector, 1.035),
            new ElevatorCommand(13.8, elevator, endEffector),
            new PivotCommand(endEffector, 0.698), 
            new ElevatorCommand(18, elevator, endEffector)
        );
    }
    
}