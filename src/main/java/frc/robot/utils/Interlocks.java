package frc.robot.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;

public class Interlocks {
    private double m_elevatorHeight;
    private double m_pivotPosition;

    public void setElevatorHeight(double height) {
        m_elevatorHeight = height;
    }

    public void setPivotPosition(double position) {
        m_pivotPosition = position;
    }

    public double clampElevatorMotorSet(double speed) {
        final Pair<Double, Double> pivotLimits = EndEffectorConstants.kSafePivotPositions.floorEntry(m_elevatorHeight)
                .getValue();

        // check if in limits
        if (m_pivotPosition > pivotLimits.getSecond() || m_pivotPosition < pivotLimits.getFirst()) {
            return ElevatorConstants.kElevatorFeedForward; // TODO: check is needed
        }

        return speed;
    }

    public double clampPivotMotorSetpoint(double setpoint) {
        final Pair<Double, Double> pivotLimits = EndEffectorConstants.kSafePivotPositions.floorEntry(m_elevatorHeight)
                .getValue();

        return MathUtil.clamp(setpoint, pivotLimits.getFirst(), pivotLimits.getSecond());
    }

    public double clampPivotMotorSet(double speed) {
        final Pair<Double, Double> pivotLimits = EndEffectorConstants.kSafePivotPositions.floorEntry(m_elevatorHeight)
                .getValue();

        if (m_pivotPosition < pivotLimits.getFirst() || m_pivotPosition > pivotLimits.getSecond()) {
            return 0; // TODO: check if needs feedforwards
        }

        return speed;
    }
}
