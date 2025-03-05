import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Map.Entry;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Pair;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.IOConstants;

public class ConstantsTest {
    @BeforeEach
    void setup() {}

    /**
     * Tests for safeguards kSafePivotPositions
     */
    @Test
    void test_SafePivotPositionsSafegaurds() {
        final double minElevator0 = EndEffectorConstants.kSafePivotPositions.firstKey();
        final double minElevator1 = EndEffectorConstants.kSafePivotPositions.higherKey(minElevator0);
        final double minElevator2 = EndEffectorConstants.kSafePivotPositions.higherKey(minElevator1);
        final double maxElevator1 = EndEffectorConstants.kSafePivotPositions.lastKey();
        final double maxElevator0 = EndEffectorConstants.kSafePivotPositions.lowerKey(maxElevator1);

        assertTrue(minElevator1 < ElevatorConstants.kElevatorBottom, "Missing minimum safegaurd pivot limit");
        assertTrue(minElevator0 < minElevator1, "Missing minimum safegaurd pivot limit");
        assertTrue(minElevator2 == ElevatorConstants.kElevatorBottom, "Pivot limits do not start at elevator bottom position");
        assertTrue(maxElevator0 > ElevatorConstants.kElevatorTop, "Missing maximum safegaurd pivot limit");
        assertTrue(maxElevator1 > maxElevator0, "Missing maximum safegaurd pivot limit");
    }

    /**
     * Tests for sane values (e.g. minimum greater than minimum)
     */
    @Test
    void test_SaneValues_nodiscrete() {
        // io
        assertTrue(IOConstants.kSlowModeScalar < 1, "Slow mode scalar not less than 1");

        // elevator
        assertTrue(ElevatorConstants.kElevatorBottom < ElevatorConstants.kElevatorTop, "Elevator min greater than max");

        // pivot
        double min = Double.MAX_VALUE;
        double max = -Double.MIN_VALUE;
        for (Entry<Double, List<Pair<Double, Double>>> entry : EndEffectorConstants.kSafePivotPositions.entrySet()) {
            min = Math.min(min, entry.getValue().get(0).getFirst());
            max = Math.max(max, entry.getValue().get(0).getSecond());

            assertTrue(entry.getValue().get(0).getFirst() < entry.getValue().get(0).getSecond(), "Invalid pivot positions");
            assertTrue(entry.getValue().get(0).getSecond() > EndEffectorConstants.kMinAlgaeExtension, "Pivot max less than algae extension");
        }
        assertTrue(min < max, "Pivot min greater than max");
        assertTrue(max < EndEffectorConstants.kPivotWraparoundPoint, "Pivot max greater than wraparound point");
    }

    /**
     * Tests if pivot limits are physically possible
     */
    @Test
    void test_SafePivotPositionsPossible_nodiscrete() {
        Entry<Double, List<Pair<Double, Double>>> limit = EndEffectorConstants.kSafePivotPositions.firstEntry();
        Pair<Double, Double> prevLimit = limit.getValue().get(0);
        
        for (limit = EndEffectorConstants.kSafePivotPositions.higherEntry(limit.getKey()); limit != null; limit = EndEffectorConstants.kSafePivotPositions.higherEntry(limit.getKey())) {
            Pair<Double, Double> curLimit = limit.getValue().get(0);
            assertTrue(
                (curLimit.getFirst() >= prevLimit.getFirst() && curLimit.getFirst() <= prevLimit.getSecond()) ||
                (curLimit.getSecond() >= prevLimit.getFirst() && curLimit.getSecond() <= prevLimit.getSecond()) ||
                (prevLimit.getFirst() >= curLimit.getFirst() && prevLimit.getFirst() <= curLimit.getSecond()) ||
                (prevLimit.getSecond() >= curLimit.getFirst() && prevLimit.getSecond() <= curLimit.getSecond())
            );
        }
    }

    /**
     * Tests for ascending order in kSafePivotPositions
     */
    @Test
    void test_SafePivotPositionsAscending() {
        Entry<Double, List<Pair<Double, Double>>> limit = EndEffectorConstants.kSafePivotPositions.firstEntry();
        double prevLimit = limit.getKey();
        
        for (limit = EndEffectorConstants.kSafePivotPositions.higherEntry(limit.getKey()); limit != null; limit = EndEffectorConstants.kSafePivotPositions.higherEntry(limit.getKey())) {
            final double currentLimit = limit.getKey();
            assertTrue(prevLimit < currentLimit, "Elevator height are not in ascending order");
            prevLimit = currentLimit;
        }
    }

    @AfterEach
    void shutdown() {}
}
