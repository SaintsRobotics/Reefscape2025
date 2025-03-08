package frc.robot.commands.scoring;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.commands.scoring.algae.AlgaeL3Command;
import frc.robot.commands.scoring.coral.CoralL3Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class L3Command extends ConditionalCommand {
    
    public L3Command(EndEffectorSubsystem endEffector, ElevatorSubsystem elevator, BooleanSupplier coralMode) {
        super(
            new CoralL3Command(endEffector, elevator),
            new AlgaeL3Command(endEffector, elevator),
            coralMode
        );
    }

}
