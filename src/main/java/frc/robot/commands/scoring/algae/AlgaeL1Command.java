package frc.robot.commands.scoring.algae;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SetpointConstants;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.commands.scoring.L4Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class AlgaeL1Command extends SequentialCommandGroup {

    public AlgaeL1Command(EndEffectorSubsystem endEffector, ElevatorSubsystem elevator) {
        addCommands(
            L4Command.reverseL4Commad(endEffector, elevator),
            new ElevatorCommand(SetpointConstants.kL1CoralHeight, elevator, endEffector),
            new PivotCommand(endEffector, SetpointConstants.kProcessorAngle)
        );
    }
    
}
