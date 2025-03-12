package frc.robot.commands.scoring.coral;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SetpointConstants;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.commands.scoring.L4Command;
import frc.robot.commands.scoring.L4Command.ElevatorReverserState;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class CoralL4Command extends SequentialCommandGroup {

    public CoralL4Command(EndEffectorSubsystem endEffector, ElevatorSubsystem elevator) {
        addCommands(
                L4Command.reverseL4Commad(endEffector, elevator), // safeguard to avoid deadlock in case already at L4.
                                                                  // TODO: maybe replace with conditional command
                new PivotCommand(endEffector, SetpointConstants.kL4CoralNoneAngle),
                new ElevatorCommand(SetpointConstants.kL3CoralHeight, elevator, endEffector),
                new PivotCommand(endEffector, SetpointConstants.kL4CoralSingleAngle),
                L4Command.getStateCommand(ElevatorReverserState.STATE_REVERSE_SINGLE),
                new ElevatorCommand(SetpointConstants.kL4CoralSingleHeight, elevator, endEffector),
                new PivotCommand(endEffector, SetpointConstants.kL4CoralDoubleAngle),
                L4Command.getStateCommand(ElevatorReverserState.STATE_REVERSE_DOUBLE),
                new ElevatorCommand(SetpointConstants.kL4CoralDoubleHeight, elevator, endEffector));
    }

}