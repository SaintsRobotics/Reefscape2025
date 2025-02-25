package frc.robot.utils;

import java.util.Map.Entry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;

public class Interlocks {
    private double m_elevatorHeight;
    private double m_pivotPosition;

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
     * clamps the setpoint to valid elevator setpoints
     * assumes elevator is currently in a valid position, otherwise undefined behavior
     * TODO: either simplify or entirely remove, this might be too slow and likely has many bugs
     * @param setpoint The desired setpoint
     * @return The clamped setpoint
     */
    public double clampElevatorMotorSetpoint(double setpoint) {
        Entry<Double, Pair<Double,Double>> pivotLimits = EndEffectorConstants.kSafePivotPositions.floorEntry(m_elevatorHeight);

        /*
         * The way this algorithm works is by incrementing the entry until we find an invalid configuration, then use the last found valid configuration
         * Assumes we start at a valid configuration
         */
        Entry<Double, Pair<Double,Double>> lastValid = pivotLimits;
        
        if (setpoint > m_elevatorHeight) { // going up
            while (m_pivotPosition >= pivotLimits.getValue().getFirst() && m_pivotPosition <= pivotLimits.getValue().getSecond()) { // check if still in limits
                // try next limit
                lastValid = pivotLimits;
                pivotLimits = EndEffectorConstants.kSafePivotPositions.higherEntry(pivotLimits.getKey());
            }
        }
        else { // going down
            while (m_pivotPosition >= pivotLimits.getValue().getFirst() && m_pivotPosition <= pivotLimits.getValue().getSecond()) { // check if still in limits
                // try next limit
                lastValid = pivotLimits;
                pivotLimits = EndEffectorConstants.kSafePivotPositions.lowerEntry(pivotLimits.getKey());
            }
        }

        /*
         * At this point, the minimum setpoint is the lastValid entry key, and the maximum setpoint is the higher entry key
         */

         final Entry<Double, Pair<Double,Double>> maxValid = EndEffectorConstants.kSafePivotPositions.higherEntry(lastValid.getKey());
         return MathUtil.clamp(setpoint, lastValid.getKey(), maxValid.getKey());
    }

    /**
     * clamps the speed to valid elevator speeds
     * should always be called before setting elevator motor
     * @param speed The desired speed
     * @return The clamped speed
     */
    public double clampElevatorMotorSet(double speed) {
        final Pair<Double, Double> pivotLimits = EndEffectorConstants.kSafePivotPositions.floorEntry(m_elevatorHeight)
                .getValue();

        // check if in limits
        if (m_pivotPosition < pivotLimits.getFirst() || m_pivotPosition > pivotLimits.getSecond() ) {
            return ElevatorConstants.kElevatorFeedForward; // TODO: check is needed
        }

        return speed;
    }

    /**
     * clamps the setpoint to valid pivot setpoints
     * @param setpoint The desired setpoint
     * @return The clamped setpoint
     */
    public double clampPivotMotorSetpoint(double setpoint) {
        final Pair<Double, Double> pivotLimits = EndEffectorConstants.kSafePivotPositions.floorEntry(m_elevatorHeight)
                .getValue();

        return MathUtil.clamp(setpoint, pivotLimits.getFirst(), pivotLimits.getSecond());
    }

    /**
     * clamps the speed to valid pivot speeds
     * should always be called before setting pivot motor
     * @param speed The desired speed
     * @return The clamped speed
     */
    public double clampPivotMotorSet(double speed) {
        final Pair<Double, Double> pivotLimits = EndEffectorConstants.kSafePivotPositions.floorEntry(m_elevatorHeight)
                .getValue();

        if (m_pivotPosition < pivotLimits.getFirst() || m_pivotPosition > pivotLimits.getSecond()) {
            return 0; // TODO: check if needs feedforwards
        }

        return speed;
    }
}
