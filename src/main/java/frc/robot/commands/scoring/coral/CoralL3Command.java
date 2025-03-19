package frc.robot.commands.scoring.coral;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.ElevatorNoPivotCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class CoralL3Command extends SequentialCommandGroup {

    public CoralL3Command(EndEffectorSubsystem endEffector, ElevatorSubsystem elevator) {
        super(
            new ParallelCommandGroup(
                new PivotCommand(endEffector, 0.36),
                new ElevatorNoPivotCommand(ElevatorConstants.kL3Height, elevator)),
            new PivotCommand(endEffector, 1.035)
        );
    }
    
}
