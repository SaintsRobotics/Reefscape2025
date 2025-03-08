package frc.robot.commands.scoring;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.commands.scoring.algae.AlgaeL4Command;
import frc.robot.commands.scoring.coral.CoralL4Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class L4Command extends ConditionalCommand {
    
    public L4Command(EndEffectorSubsystem endEffector, ElevatorSubsystem elevator, BooleanSupplier coralMode) {
        super(
            new CoralL4Command(endEffector, elevator),
            new AlgaeL4Command(endEffector, elevator),
            coralMode
        );
    }

}
