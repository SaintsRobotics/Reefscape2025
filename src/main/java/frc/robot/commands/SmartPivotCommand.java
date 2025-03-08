package frc.robot.commands;

import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.EndEffectorSubsystem;

public class SmartPivotCommand extends Command {
    private final EndEffectorSubsystem m_endEffector;
    private final DoubleSupplier m_elevatorHeight;

    public SmartPivotCommand(EndEffectorSubsystem endEffectorSubsystem, DoubleSupplier elevatorHeight) {
        addRequirements(endEffectorSubsystem);

        m_endEffector = endEffectorSubsystem;
        m_elevatorHeight = elevatorHeight;
    }

    @Override
    public void execute() {
        final List<Pair<Double, Double>> currentLimit = EndEffectorConstants.kSafePivotPositions
                .floorEntry(m_elevatorHeight.getAsDouble()).getValue();

        boolean needsClamp = true;
        double pivotPosition = m_endEffector.getSetpoint();
        for (Pair<Double, Double> limit : currentLimit) {
            if (pivotPosition >= limit.getFirst() && pivotPosition <= limit.getSecond()) {
                needsClamp = false;
            }
        }

        if (needsClamp) {
            pivotPosition = MathUtil.clamp(pivotPosition, currentLimit.get(0).getFirst(),
                    currentLimit.get(0).getFirst());
        }

        m_endEffector.pivotTo(pivotPosition);
    }
}
