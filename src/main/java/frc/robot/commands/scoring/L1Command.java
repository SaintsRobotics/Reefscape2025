package frc.robot.commands.scoring;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.commands.scoring.algae.AlgaeProcessorCommand;
import frc.robot.commands.scoring.coral.CoralL1Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class L1Command extends ConditionalCommand {
    
    public L1Command(EndEffectorSubsystem endEffector, ElevatorSubsystem elevator, BooleanSupplier coralMode) {
        super(
            new CoralL1Command(endEffector, elevator),
            new AlgaeProcessorCommand(endEffector, elevator),
            coralMode
        );
    }

}
