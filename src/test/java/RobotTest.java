import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

import java.time.Duration;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.robot.Robot;


public class RobotTest {
    @BeforeEach
    void setup() {}

    /**
     * Tests for robot code crashing within the first 5 seconds of the code starting
     */
    @Test
    void test_Robot() {
        try (Robot robot = new Robot()) {
            assertDoesNotThrow(() -> {
                final Duration timeout = Duration.ofMillis(5000);
                final ExecutorService executor = Executors.newSingleThreadExecutor();
                final Future<?> future = executor.submit(() -> {
                    robot.startCompetition();
                });

                try {
                    future.get(timeout.toMillis(), TimeUnit.MILLISECONDS);
                }
                catch (TimeoutException e) {
                    future.cancel(true);
                }
                executor.shutdown();
            }, "Robot code crashed");
        }
    }

    @AfterEach
    void shutdown() {}
}
