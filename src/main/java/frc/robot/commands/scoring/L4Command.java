package frc.robot.commands.scoring;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SetpointConstants;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.commands.scoring.algae.AlgaeL4Command;
import frc.robot.commands.scoring.coral.CoralL4Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.utils.CommandUtils;

public class L4Command extends ConditionalCommand {
    public static enum ElevatorReverserState {
        STATE_REVERSE_NONE,
        STATE_REVERSE_SINGLE,
        STATE_REVERSE_DOUBLE;
    }

    private static ElevatorReverserState m_state = ElevatorReverserState.STATE_REVERSE_NONE;
    
    public L4Command(EndEffectorSubsystem endEffector, ElevatorSubsystem elevator, BooleanSupplier coralMode) {
        super(
            new CoralL4Command(endEffector, elevator),
            new AlgaeL4Command(endEffector, elevator),
            coralMode
        );
    }

    public static Command getStateCommand(ElevatorReverserState state) {
        return new InstantCommand(() -> m_state = state);
    }

    public static Command reverseL4Commad(EndEffectorSubsystem endEffector, ElevatorSubsystem elevator) {
        return CommandUtils.generateTripleConditionalCommand(
            new SequentialCommandGroup(
                ),
                new SequentialCommandGroup(
                    new PivotCommand(endEffector, SetpointConstants.kL4CoralSingleAngle),
                    new ElevatorCommand(SetpointConstants.kL3AlgaeHeight, elevator, endEffector),
                    getStateCommand(ElevatorReverserState.STATE_REVERSE_NONE)
                ),
                new SequentialCommandGroup(
                    new PivotCommand(endEffector, SetpointConstants.kL4CoralDoubleAngle),
                    new ElevatorCommand(SetpointConstants.kL4CoralSingleHeight, elevator, endEffector),
                    getStateCommand(ElevatorReverserState.STATE_REVERSE_SINGLE),
                    new PivotCommand(endEffector, SetpointConstants.kL4CoralSingleAngle),
                    new ElevatorCommand(SetpointConstants.kL3AlgaeHeight, elevator, endEffector),
                    getStateCommand(ElevatorReverserState.STATE_REVERSE_NONE)
                ),
                () -> m_state == ElevatorReverserState.STATE_REVERSE_NONE,
                () -> m_state == ElevatorReverserState.STATE_REVERSE_SINGLE);
    }
}
