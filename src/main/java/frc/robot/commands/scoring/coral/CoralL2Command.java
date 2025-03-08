package frc.robot.commands.scoring.coral;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class CoralL2Command extends SequentialCommandGroup {

    public CoralL2Command(EndEffectorSubsystem endEffector, ElevatorSubsystem elevator) {
        addCommands(
            new ConditionalCommand(
                new UndoCoralL4Command(endEffector, elevator),
                new PivotCommand(endEffector, 1.3),
                () -> elevator.getCurrentHeight() >= 12),
            // new PivotCommand(m_endEffector, 1.3),
            new ElevatorCommand(ElevatorConstants.kL2Height, elevator, endEffector),
            new PivotCommand(endEffector, 0.717)
        );
    }
    
}
