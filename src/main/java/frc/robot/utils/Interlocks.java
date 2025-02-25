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

    public double clampElevatorMotorSet(double in) {
        final Pair<Double, Double> pivotLimits = EndEffectorConstants.kSafePivotPositions.floorEntry(m_elevatorHeight).getValue();
        
        // check if in limits
        if (m_pivotPosition > pivotLimits.getSecond() || m_pivotPosition < pivotLimits.getFirst()) {
            return ElevatorConstants.kElevatorFeedForward; //TODO: check is needed
        }

        return in; //TODO: implement speed limiters
    }

    public double clampPivotMotorSet(double in) {
        final Pair<Double, Double> pivotLimits = EndEffectorConstants.kSafePivotPositions.floorEntry(m_elevatorHeight).getValue();
        
        return MathUtil.clamp(in, pivotLimits.getFirst(), pivotLimits.getSecond());
    }
}
