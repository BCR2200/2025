package frc.robot.timing;

import java.util.concurrent.ConcurrentHashMap;

class TimingUtilsTest {

    private static final String NAMESPACE = "TimingUtils/";

    public static void main(String[] args) {
        TimingUtilsTest test = new TimingUtilsTest();
        test.testLogDurationCorrectlyMeasuresTaskExecutionTime();
        test.testLogDurationHandlesZeroDurationTask();
        test.testLogDurationAddsEntryToLabelEntryNameMap();
        System.out.println("All tests passed!");
    }

    /**
     * Tests the `TimingUtils` utility class, focusing on the `logDuration` method.
     */
    void testLogDurationCorrectlyMeasuresTaskExecutionTime() {
        // Arrange
        initializeTest();
        String label = "timedTask";
        Runnable task = simulateTask(0.1); // Task runs for 100ms
        SimulatedTimer.setSimulatedTime(1.0); // Start time

        // Act
        TimingUtils.logDuration(label, task);

        // Assert
        double loggedDuration = TestSignalLogger.getLoggedValue(NAMESPACE + label);
        if (Math.abs(loggedDuration - 0.1) > 0.001) {
            throw new AssertionError("Expected 0.1 but got " + loggedDuration);
        }
    }

    void testLogDurationHandlesZeroDurationTask() {
        // Arrange
        initializeTest();
        String label = "zeroTask";
        Runnable task = () -> {
        }; // No-op task
        SimulatedTimer.setSimulatedTime(2.0); // Start and end time are the same

        // Act
        TimingUtils.logDuration(label, task);

        // Assert
        double loggedDuration = TestSignalLogger.getLoggedValue(NAMESPACE + label);
        if (Math.abs(loggedDuration - 0.0) > 0.001) {
            throw new AssertionError("Expected 0.0 but got " + loggedDuration);
        }
    }

    void testLogDurationAddsEntryToLabelEntryNameMap() {
        // Arrange
        initializeTest();
        String label = "uniqueTask";
        Runnable task = () -> {
        };
        SimulatedTimer.setSimulatedTime(3.0);

        // Act
        TimingUtils.logDuration(label, task);

        // Assert
        String expectedEntryName = NAMESPACE + label;
        String actualEntryName = TimingUtils.labelEntryNameMap.get(label);
        if (!expectedEntryName.equals(actualEntryName)) {
            throw new AssertionError("Expected " + expectedEntryName + " but got " + actualEntryName);
        }
    }

    private void initializeTest() {
        TestSignalLogger.reset();
        SimulatedTimer.reset();
        TimingUtils.labelEntryNameMap.clear();
        // Replace real implementation with mock implementations, for easier testing.
        TimingUtils.setTimeSupplier(SimulatedTimer::getSimulatedTime);
        TimingUtils.setValueConsumer(TestSignalLogger::writeDouble);
    }

    private Runnable simulateTask(double durationInSeconds) {
        return () -> {
            // Simulate time passing by advancing the sim timer
            SimulatedTimer.setSimulatedTime(SimulatedTimer.getSimulatedTime() + durationInSeconds);
        };
    }

    public static class SimulatedTimer {
        private static double simulatedTime;

        public static void reset() {
            simulatedTime = 0.0;
        }

        public static void setSimulatedTime(double time) {
            simulatedTime = time;
        }

        public static double getSimulatedTime() {
            return simulatedTime;
        }
    }

    public static class TestSignalLogger {
        private static final ConcurrentHashMap<String, Double> loggedValues = new ConcurrentHashMap<>();

        public static void writeDouble(String entryName, double value) {
            loggedValues.put(entryName, value);
        }

        public static double getLoggedValue(String entryName) {
            return loggedValues.getOrDefault(entryName, -1.0);
        }

        public static void reset() {
            loggedValues.clear();
        }
    }
}