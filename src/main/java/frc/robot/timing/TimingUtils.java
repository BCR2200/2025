package frc.robot.timing;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.Timer;

import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

public class TimingUtils {

    // Use a supplier to inject the timing method
    private static Supplier<Double> timeSupplier = Timer::getFPGATimestamp;

    public static void setTimeSupplier(Supplier<Double> supplier) {
        timeSupplier = supplier;
    }

    // Use a consumer to inject the logging method
    private static BiConsumer<String, Double> valueConsumer = SignalLogger::writeDouble;

    public static void setValueConsumer(BiConsumer<String, Double> consumer) {
        valueConsumer = consumer;
    }

    static final ConcurrentMap<String, String> labelEntryNameMap = new ConcurrentHashMap<>();

    public static double currentTimeMillis() {
        return timeSupplier.get();
    }

    public static void logDuration(String label, Runnable task) {
        // Get timestamp from FPGA
        double startTime = currentTimeMillis();

        try {
            task.run();
        } finally {
            double endTime = currentTimeMillis();
            double delta = endTime - startTime;
            // We want to capture all timings under a "TimingUtils/" namespace.
            // It is inefficient to generate a string each time, so we should generate it only
            // if it isn't in the cache, otherwise using the cached value.
            String labelEntryName = labelEntryNameMap.computeIfAbsent(label, key -> "TimingUtils/" + key);
            valueConsumer.accept(labelEntryName, delta);
        }
    }
}
