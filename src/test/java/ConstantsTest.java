import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Map.Entry;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Pair;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;

public class ConstantsTest {
    @BeforeEach
    void setup() {}

    /**
     * Tests for safeguards kSafePivotPositions
     */
    @Test
    void test_SafePivotPositionsSafegaurds() {
        final double minElevator = EndEffectorConstants.kSafePivotPositions.firstKey();
        final double minElevator2 = EndEffectorConstants.kSafePivotPositions.higherKey(minElevator);
        final double maxElevator = EndEffectorConstants.kSafePivotPositions.lastKey();

        assertTrue(minElevator < ElevatorConstants.kElevatorBottom, "Missing minimum safegaurd pivot limit");
        assertTrue(minElevator2 == ElevatorConstants.kElevatorBottom, "Pivot limits do not start at elevator bottom position");
        assertTrue(maxElevator > ElevatorConstants.kElevatorTop, "Missing maximum safegaurd pivot limit");
    }

    /**
     * Tests for ascending order in kSafePivotPositions
     */
    @Test
    void test_SafePivotPositionsContinuity() {
        Entry<Double, Pair<Double, Double>> limit = EndEffectorConstants.kSafePivotPositions.firstEntry();
        double prevLimit = limit.getKey();
        
        for (limit = EndEffectorConstants.kSafePivotPositions.higherEntry(limit.getKey()); limit != null; limit = EndEffectorConstants.kSafePivotPositions.higherEntry(limit.getKey())) {
            final double currentLimit = limit.getKey();
            assertTrue(prevLimit < currentLimit, "Pivot limits are not in ascending order");
            prevLimit = currentLimit;
        }
    }

    @AfterEach
    void shutdown() {}
}
