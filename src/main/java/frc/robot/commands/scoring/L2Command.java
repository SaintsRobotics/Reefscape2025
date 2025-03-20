package frc.robot.commands.scoring;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.commands.scoring.algae.AlgaeL2Command;
import frc.robot.commands.scoring.coral.CoralL2Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class L2Command extends ConditionalCommand {
    
    public L2Command(EndEffectorSubsystem endEffector, ElevatorSubsystem elevator, BooleanSupplier coralMode) {
        super(
            new CoralL2Command(endEffector, elevator),
            new AlgaeL2Command(endEffector, elevator),
            coralMode
        );
    }

}
