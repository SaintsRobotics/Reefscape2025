package frc.robot.commands.scoring.algae;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SetpointConstants;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.commands.scoring.L4Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class AlgaeL2Command extends SequentialCommandGroup {

    public AlgaeL2Command(EndEffectorSubsystem endEffector, ElevatorSubsystem elevator) {
        addCommands(
            L4Command.reverseL4Commad(endEffector, elevator),
            new PivotCommand(endEffector, SetpointConstants.kAlgaeOutAngle),
            new ElevatorCommand(SetpointConstants.kL2AlgaeHeight, elevator, endEffector), 
            new PivotCommand(endEffector, SetpointConstants.kL2AlgaeAngle)
        );
    }
    
}
