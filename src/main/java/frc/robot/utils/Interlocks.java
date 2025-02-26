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
     * @param setpoint The desired setpoint
     * @return The clamped setpoint
     */
    public double clampElevatorMotorSetpoint(double setpoint) {
        // first clamp setpoint to extension limits;
        setpoint = MathUtil.clamp(setpoint, ElevatorConstants.kElevatorBottom, ElevatorConstants.kElevatorTop);

        final Entry<Double, Pair<Double, Double>> currentLimit = EndEffectorConstants.kSafePivotPositions.floorEntry(m_elevatorHeight);
        final Entry<Double, Pair<Double, Double>> setpointLimts = EndEffectorConstants.kSafePivotPositions.floorEntry(setpoint);

        Entry<Double, Pair<Double, Double>> iteratedLimit = currentLimit;
        Entry<Double, Pair<Double, Double>> lastValidLimit;

        /*
         * This algorithm works by iterating each limit until we find a limit where m_pivotPosition does not satisfy
         * The limit immediatly before this invalid limit is the max/min setpoint
         */
        while (iteratedLimit != setpointLimts) {
            // save previous valid since that must be valid
            lastValidLimit = iteratedLimit;

            // determine direction
            if (iteratedLimit.getKey() < setpointLimts.getKey()) { // going up
                iteratedLimit = EndEffectorConstants.kSafePivotPositions.higherEntry(iteratedLimit.getKey());
            }
            else { // going down
                iteratedLimit = EndEffectorConstants.kSafePivotPositions.lowerEntry(iteratedLimit.getKey());
            }

            // check if setpoint is valid in iteratedLimit
            if (m_pivotPosition < iteratedLimit.getValue().getFirst() || m_pivotPosition > iteratedLimit.getValue().getSecond()) { // out of bounds
                // clamp to between lastValidLimit and one higher
                final Entry<Double, Pair<Double, Double>> oneHigherLimit = EndEffectorConstants.kSafePivotPositions.higherEntry(lastValidLimit.getKey());

                return MathUtil.clamp(setpoint, lastValidLimit.getKey(), oneHigherLimit.getKey());
            }
        }

        // if the code reaches here, that means there was no issue with the setpoint
        return setpoint;
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
