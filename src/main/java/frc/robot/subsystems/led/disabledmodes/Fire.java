package frc.robot.subsystems.led.disabledmodes;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.led.LEDDrawer;
import frc.robot.subsystems.led.Strip;

public class Fire extends LEDDrawer {
    private double[] noise = new double[60]; // One noise value per LED
    private double time = 0;
    private static final double NOISE_SPEED = 0.1;
    private static final double NOISE_SCALE = 0.5;
    private static final double FLAME_HEIGHT = 0.7; // How high the flame goes (0-1)

    public Fire(LEDSubsystem subsystem, AddressableLED ledStrip, AddressableLEDBuffer buffer) {
        super(subsystem, ledStrip, buffer);
        // Initialize noise array with random values
        for (int i = 0; i < noise.length; i++) {
            noise[i] = Math.random();
        }
    }

    @Override
    public void draw() {
        time += NOISE_SPEED;
        
        // Update noise values using Perlin-like noise
        for (int i = 0; i < noise.length; i++) {
            noise[i] = (noise[i] + Math.sin(time + i * NOISE_SCALE) * 0.1) % 1.0;
        }

        // Draw fire effect on each strip
        for (Strip strip : susystem.strips) {
            for (int i = 0; i < strip.numLEDs; i++) {
                // Calculate normalized position (0-1)
                double position = (double) i / strip.numLEDs;
                
                // Calculate flame intensity based on position and noise
                double flameIntensity = Math.max(0, 1 - position / FLAME_HEIGHT);
                double noiseValue = noise[strip.start + strip.direction * i];
                
                // Combine position-based intensity with noise
                double finalIntensity = flameIntensity * (0.7 + 0.3 * noiseValue);
                
                // Create fire colors (red to yellow gradient)
                Color color;
                if (finalIntensity > 0.5) {
                    // Yellow to white for the hottest part
                    double t = (finalIntensity - 0.5) * 2;
                    color = new Color(
                        (int)(255 * (0.8 + 0.2 * t)),
                        (int)(255 * (0.4 + 0.6 * t)),
                        (int)(255 * (0.1 + 0.9 * t))
                    );
                } else {
                    // Red to yellow for the cooler part
                    double t = finalIntensity * 2;
                    color = new Color(
                        (int)(255 * (0.8 + 0.2 * t)),
                        (int)(255 * (0.2 + 0.2 * t)),
                        (int)(255 * 0.1)
                    );
                }
                
                susystem.safeSetLED(strip.start + strip.direction * i, color);
            }
        }
    }

    @Override
    public int sleepInterval() {
        return 20;
    }
} 