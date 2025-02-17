package frc.robot.timing;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;

import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;
import java.util.function.Supplier;

public class TimingUtils {

    // Use a supplier to inject the timing method
    private static Supplier<Double> timeSupplier = Timer::getFPGATimestamp;

    public static void setTimeSupplier(Supplier<Double> supplier) {
        timeSupplier = supplier;
    }

    // Use a consumer to inject the logging method
    private static Supplier<DataLog> logGetter = DataLogManager::getLog;

    public static void setLogGetter(Supplier<DataLog> logGetter) {
        TimingUtils.logGetter = logGetter;
    }

    static final ConcurrentMap<String, DoubleLogEntry> labelToLogEntryMap = new ConcurrentHashMap<>();

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
            DoubleLogEntry entry = labelToLogEntryMap.computeIfAbsent(label, key -> {
                String fullKey = "TimingUtils/" + key;
              return new DoubleLogEntry(logGetter.get(), fullKey);
            });
            entry.append(delta);
        }
    }
}
