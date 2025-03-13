package frc.robot.subsystems.led.disabledmodes;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.led.LEDDrawer;
import frc.robot.subsystems.led.Strip;

public class Plasma extends LEDDrawer {
    private double time = 0;
    private static final double SPEED = 0.02;
    private static final double BRIGHTNESS = 0.6;
    
    // Different frequencies for each color channel to create interesting patterns
    private static final double RED_FREQ = 0.5;
    private static final double GREEN_FREQ = 0.7;
    private static final double BLUE_FREQ = 0.3;
    
    // Different phase shifts for each color channel
    private static final double RED_PHASE = 0.0;
    private static final double GREEN_PHASE = 2.094; // 2π/3
    private static final double BLUE_PHASE = 4.189; // 4π/3

    public Plasma(LEDSubsystem subsystem, AddressableLED ledStrip, AddressableLEDBuffer buffer) {
        super(subsystem, ledStrip, buffer);
    }

    @Override
    public void draw() {
        time += SPEED;
        
        // Draw plasma effect on each strip
        for (Strip strip : susystem.strips) {
            for (int i = 0; i < strip.numLEDs; i++) {
                // Calculate normalized position (0-1)
                double position = (double) i / strip.numLEDs;
                
                // Calculate phase for this LED
                double phase = position * 2 * Math.PI;
                
                // Calculate each color channel using sine waves with different frequencies and phases
                double red = (Math.sin(phase * RED_FREQ + time + RED_PHASE) + 1) / 2;
                double green = (Math.sin(phase * GREEN_FREQ + time + GREEN_PHASE) + 1) / 2;
                double blue = (Math.sin(phase * BLUE_FREQ + time + BLUE_PHASE) + 1) / 2;
                
                // Add some noise to make it more organic
                double noise = (Math.sin(position * 10 + time * 2) + 1) / 2;
                red = (red + noise * 0.2) / 1.2;
                green = (green + noise * 0.2) / 1.2;
                blue = (blue + noise * 0.2) / 1.2;
                
                // Create the plasma color
                Color plasmaColor = new Color(
                    (int)(255 * red * BRIGHTNESS),
                    (int)(255 * green * BRIGHTNESS),
                    (int)(255 * blue * BRIGHTNESS)
                );
                
                susystem.safeSetLED(strip.start + strip.direction * i, plasmaColor);
            }
        }
    }

    @Override
    public int sleepInterval() {
        return 20;
    }
} 