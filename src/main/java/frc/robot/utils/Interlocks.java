package frc.robot.utils;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;

public class Interlocks {
    private double m_elevatorHeight = 0;
    private double m_pivotPosition = 0;
    private boolean m_holdingAlgea = false;

    /**
     * Updates the internal logic with the latest elevator height
     * @param height The latest elevator height
     */
    public void setElevatorHeight(double height) {
        m_elevatorHeight = height;
    }

    /**
     * Updates the internal logic with the latest pivot position
     * @param position The latest pivot position
     */
    public void setPivotPosition(double position) {
        m_pivotPosition = position;
    }

    /**
     * Updates the internal logic with the latest algae holding status
     * @param isHolding True if holding algae. False otherwise
     */
    public void setAlgeaHolding(boolean isHolding) {
        m_holdingAlgea = isHolding;
    }

    /**
     * clamps the speed to valid elevator speeds
     * should always be called before setting elevator motor
     * @param speed The desired speed
     * @return The clamped speed
     */
    public double clampElevatorMotorSet(double speed) {
        final List<Pair<Double, Double>> pivotLimits = EndEffectorConstants.kSafePivotPositions.floorEntry(m_elevatorHeight).getValue();

        speed = MathUtil.clamp(speed, ElevatorConstants.kElevatorDownMaxSpeed, ElevatorConstants.kElevatorUpMaxSpeed);

        // check if within physical limits
        if (m_elevatorHeight < ElevatorConstants.kElevatorBottom && speed < 0) {
            return ElevatorConstants.kElevatorFeedForward; // TODO: check is needed
        }
        if (m_elevatorHeight > ElevatorConstants.kElevatorTop && speed > 0) {
            return ElevatorConstants.kElevatorFeedForward; // TODO: check is needed
        }
        if (m_elevatorHeight < ElevatorConstants.kLowHeightSlowdownThreshold && speed < 0) {
            return MathUtil.clamp(speed, ElevatorConstants.kLowHeightSlowdownMaxSpeed, 0);
        }

        for (Pair<Double, Double> limit : pivotLimits) {
            if (m_pivotPosition >= limit.getFirst() && m_pivotPosition <= limit.getSecond()) {
                return speed;
            }
        }
        
        return ElevatorConstants.kElevatorFeedForward;
    }

    /**
     * clamps the speed to valid pivot speeds
     * should always be called before setting pivot motor
     * @param speed The desired speed
     * @return The clamped speed
     */
    public double clampPivotMotorSet(double speed) {
        final List<Pair<Double, Double>> pivotLimits = EndEffectorConstants.kSafePivotPositions
                .floorEntry(m_elevatorHeight).getValue();

        speed = MathUtil.clamp(speed, EndEffectorConstants.kPivotMaxSpeedExtend,
                EndEffectorConstants.kPivotMaxSpeedRetract);

        if (false && m_holdingAlgea && m_pivotPosition < EndEffectorConstants.kMinAlgaeExtension && speed > 0) {
            return EndEffectorConstants.kPivotFeedForwards;
        }

        Pair<Double, Double> closestLimit = null;
        double minDist = Double.MAX_VALUE;

        for (Pair<Double, Double> limit : pivotLimits) {
            if ((m_pivotPosition >= limit.getFirst()) && (m_pivotPosition <= limit.getSecond())) {
                return speed;
            }

            final double min = Math.min(Math.abs(m_pivotPosition - limit.getFirst()),
                    Math.abs(m_pivotPosition - limit.getSecond()));
            if (min < minDist) {
                minDist = min;
                closestLimit = limit;
            }
        }

        if ((m_pivotPosition < closestLimit.getFirst() && speed < 0) || (m_pivotPosition > closestLimit.getSecond() && speed > 0)) {
            return speed;
        }

        return EndEffectorConstants.kPivotFeedForwards;
    }
}
