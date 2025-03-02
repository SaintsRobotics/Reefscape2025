import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import java.lang.Math;
import java.util.NavigableMap;

import edu.wpi.first.math.Pair;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.utils.Interlocks;

public class InterlocksTest {
    // private NavigableMap<Double, Pair<Double, Double>> m_safePivots = EndEffectorConstants.kSafePivotPositions;
    // private static final double maxSpeedUp = ElevatorConstants.kElevatorUpMaxSpeed; // store here for less typing
    // private static final double maxSpeedDown = ElevatorConstants.kElevatorDownMaxSpeed;
    // private static final double maxSpeed = EndEffectorConstants.kPivotMaxSpeedRetract;
    // private Interlocks m_interlocks;

    // private static void assertExactlyEquals(double a, double b) {
    //     assertEquals(0, a - b, Math.ulp(a - b), String.format("Interlocks failed, Expected %f but got %f", b, a));
    // }

    // @BeforeEach
    // void setup() {
    //     m_safePivots.clear();
    //     m_safePivots.put(-Double.MAX_VALUE, Pair.of(0.0, 1.0));
    //     m_safePivots.put(0.0, Pair.of(0.0, 1.0));
    //     m_safePivots.put(1.0, Pair.of(1.0, 10.0));
    //     m_safePivots.put(2.0, Pair.of(11.0, 100.0));
    //     m_safePivots.put(Double.MAX_VALUE, Pair.of(10.0, 100.0));
    //     m_interlocks = new Interlocks();
    // }

    // /**
    //  * Tests for correct elevator clamp behavior
    //  */
    // @Test
    // void test_ElevatorClamp_NoClamp() {
    //     m_interlocks.setElevatorHeight(0);
    //     m_interlocks.setPivotPosition(0);

        
    //     assertExactlyEquals(m_interlocks.clampElevatorMotorSet(0), 0);
    //     assertExactlyEquals(m_interlocks.clampElevatorMotorSet(maxSpeedUp/2), maxSpeedUp/2);
    //     assertExactlyEquals(m_interlocks.clampElevatorMotorSet(maxSpeedUp*2), maxSpeedUp);
    // }

    // /**
    //  * Tests for correct elevator clamp behavior
    //  */
    // @Test
    // void test_ElevatorClamp_Clamp() {
    //     m_interlocks.setElevatorHeight(1);
    //     m_interlocks.setPivotPosition(0);

    //     assertExactlyEquals(m_interlocks.clampElevatorMotorSet(0), ElevatorConstants.kElevatorFeedForward);
    //     assertExactlyEquals(m_interlocks.clampElevatorMotorSet(maxSpeedUp/2), ElevatorConstants.kElevatorFeedForward);
    //     assertExactlyEquals(m_interlocks.clampElevatorMotorSet(maxSpeedUp*2), ElevatorConstants.kElevatorFeedForward);
    //     assertExactlyEquals(m_interlocks.clampElevatorMotorSet(maxSpeedDown*2), ElevatorConstants.kElevatorFeedForward);
    // }

    // /**
    //  * Tests for correct elevator clamp behavior
    //  */
    // @Test
    // void test_ElevatorClamp_Edge() {
    //     m_interlocks.setElevatorHeight(1);
    //     m_interlocks.setPivotPosition(10);

    //     assertExactlyEquals(m_interlocks.clampElevatorMotorSet(maxSpeedUp/2), maxSpeedUp/2);

    //     m_interlocks.setElevatorHeight(2);
    //     m_interlocks.setPivotPosition(10);

    //     assertExactlyEquals(m_interlocks.clampElevatorMotorSet(maxSpeedUp/2), ElevatorConstants.kElevatorFeedForward);
    // }

    // /**
    //  * Tests for correct elevator clamp behavior
    //  */
    // @Test
    // void test_ElevatorClamp_Many() {
    //     m_interlocks.setElevatorHeight(0);
    //     m_interlocks.setPivotPosition(0);

    //     assertExactlyEquals(m_interlocks.clampElevatorMotorSet(0), 0);
    //     assertExactlyEquals(m_interlocks.clampElevatorMotorSet(maxSpeedUp/2), maxSpeedUp/2);

    //     m_interlocks.setElevatorHeight(1);
    //     assertExactlyEquals(m_interlocks.clampElevatorMotorSet(0), ElevatorConstants.kElevatorFeedForward);
    //     assertExactlyEquals(m_interlocks.clampElevatorMotorSet(maxSpeedDown/2), ElevatorConstants.kElevatorFeedForward);

    //     m_interlocks.setPivotPosition(99);
    //     assertExactlyEquals(m_interlocks.clampElevatorMotorSet(maxSpeedUp/2), ElevatorConstants.kElevatorFeedForward);
    //     assertExactlyEquals(m_interlocks.clampElevatorMotorSet(maxSpeedDown*2), ElevatorConstants.kElevatorFeedForward);

    //     m_interlocks.setElevatorHeight(1.99);

    //     assertExactlyEquals(m_interlocks.clampElevatorMotorSet(0), ElevatorConstants.kElevatorFeedForward);
    //     assertExactlyEquals(m_interlocks.clampElevatorMotorSet(maxSpeedDown*2), ElevatorConstants.kElevatorFeedForward);

    //     m_interlocks.setElevatorHeight(2);

    //     assertExactlyEquals(m_interlocks.clampElevatorMotorSet(0), 0.0);
    //     assertExactlyEquals(m_interlocks.clampElevatorMotorSet(maxSpeedUp), maxSpeedUp);
    // }

    // /**
    //  * Tests for correct pivot clamp behavior
    //  */
    // @Test
    // @Disabled
    // void test_PivotClamp_NoClamp() {
    //     m_interlocks.setElevatorHeight(0);
    //     m_interlocks.setPivotPosition(0);

        
    //     assertExactlyEquals(m_interlocks.clampPivotMotorSet(0), 0);
    //     assertExactlyEquals(m_interlocks.clampPivotMotorSet(maxSpeed), maxSpeed);
    //     assertExactlyEquals(m_interlocks.clampPivotMotorSet(maxSpeed*2), maxSpeed);
    //     assertExactlyEquals(m_interlocks.clampPivotMotorSet(-maxSpeed*2), -maxSpeed);
    // }

    // /**
    //  * Tests for correct pivot clamp behavior
    //  */
    // @Test
    // @Disabled
    // void test_PivotClamp_Clamp() {
    //     m_interlocks.setElevatorHeight(1);
    //     m_interlocks.setPivotPosition(0);

    //     assertExactlyEquals(m_interlocks.clampPivotMotorSet(0), 0);
    //     assertExactlyEquals(m_interlocks.clampPivotMotorSet(maxSpeed), 0);
    //     assertExactlyEquals(m_interlocks.clampPivotMotorSet(maxSpeed*2), 0);
    //     assertExactlyEquals(m_interlocks.clampPivotMotorSet(-maxSpeed*2), 0);
    // }

    // /**
    //  * Tests for correct pivot clamp behavior
    //  */
    // @Test
    // @Disabled
    // void test_PivotClamp_Edge() {
    //     m_interlocks.setElevatorHeight(1);
    //     m_interlocks.setPivotPosition(10);

    //     assertExactlyEquals(m_interlocks.clampPivotMotorSet(maxSpeed), maxSpeed);

    //     m_interlocks.setElevatorHeight(2);
    //     m_interlocks.setPivotPosition(10);

    //     assertExactlyEquals(m_interlocks.clampPivotMotorSet(maxSpeed), 0);
    // }

    // /**
    //  * Tests for correct pivot clamp behavior
    //  */
    // @Test
    // @Disabled
    // void test_PivotClamp_Many() {
    //     m_interlocks.setElevatorHeight(0);
    //     m_interlocks.setPivotPosition(0);

    //     assertExactlyEquals(m_interlocks.clampPivotMotorSet(0), 0);
    //     assertExactlyEquals(m_interlocks.clampPivotMotorSet(maxSpeed*2), maxSpeed);

    //     m_interlocks.setElevatorHeight(1);
    //     assertExactlyEquals(m_interlocks.clampPivotMotorSet(0), 0);
    //     assertExactlyEquals(m_interlocks.clampPivotMotorSet(-maxSpeed), 0);

    //     m_interlocks.setPivotPosition(99);
    //     assertExactlyEquals(m_interlocks.clampPivotMotorSet(maxSpeed/2), 0);
    //     assertExactlyEquals(m_interlocks.clampPivotMotorSet(-maxSpeed*2), 0);

    //     m_interlocks.setElevatorHeight(1.99);

    //     assertExactlyEquals(m_interlocks.clampPivotMotorSet(0), 0);
    //     assertExactlyEquals(m_interlocks.clampPivotMotorSet(maxSpeed), 0);

    //     m_interlocks.setElevatorHeight(2);

    //     assertExactlyEquals(m_interlocks.clampPivotMotorSet(0), 0);
    //     assertExactlyEquals(m_interlocks.clampPivotMotorSet(maxSpeed/2), maxSpeed/2);
    // }

    // @AfterEach
    // void shutdown() {}
}
