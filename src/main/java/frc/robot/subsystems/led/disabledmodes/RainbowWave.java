package frc.robot.subsystems.led.disabledmodes;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.led.LEDDrawer;
import frc.robot.subsystems.led.Strip;

public class RainbowWave extends LEDDrawer {
    private double hue = 0;
    private double waveOffset = 0;
    private static final double HUE_STEP = 0.02;
    private static final double WAVE_STEP = 0.05;
    private static final double WAVE_LENGTH = 0.5;

    public RainbowWave(LEDSubsystem subsystem, AddressableLED ledStrip, AddressableLEDBuffer buffer) {
        super(subsystem, ledStrip, buffer);
    }

    @Override
    public void draw() {
        // Update hue and wave offset
        hue = (hue + HUE_STEP) % 1.0;
        waveOffset = (waveOffset + WAVE_STEP) % 1.0;

        // Set background to alliance color
        susystem.setColour(susystem.fullStrip, susystem.allianceColor);

        // Draw rainbow wave on each strip
        for (Strip strip : susystem.strips) {
            for (int i = 0; i < strip.numLEDs; i++) {
                // Calculate wave position (0 to 1)
                double position = (double) i / strip.numLEDs;
                // Calculate wave value (-1 to 1)
                double wave = Math.sin(2 * Math.PI * (position + waveOffset) / WAVE_LENGTH);
                // Convert wave to brightness (0 to 1)
                double brightness = (wave + 1) / 2;
                
                // Convert HSV to RGB
                Color color = hsvToRgb(hue, 1.0, brightness);
                susystem.safeSetLED(strip.start + strip.direction * i, color);
            }
        }
    }

    @Override
    public int sleepInterval() {
        return 20;
    }

    private Color hsvToRgb(double h, double s, double v) {
        int r, g, b;
        int i = (int) (h * 6);
        double f = h * 6 - i;
        double p = v * (1 - s);
        double q = v * (1 - f * s);
        double t = v * (1 - (1 - f) * s);

        switch (i % 6) {
            case 0:
                r = (int) (v * 255);
                g = (int) (t * 255);
                b = (int) (p * 255);
                break;
            case 1:
                r = (int) (q * 255);
                g = (int) (v * 255);
                b = (int) (p * 255);
                break;
            case 2:
                r = (int) (p * 255);
                g = (int) (v * 255);
                b = (int) (t * 255);
                break;
            case 3:
                r = (int) (p * 255);
                g = (int) (q * 255);
                b = (int) (v * 255);
                break;
            case 4:
                r = (int) (t * 255);
                g = (int) (p * 255);
                b = (int) (v * 255);
                break;
            default:
                r = (int) (v * 255);
                g = (int) (p * 255);
                b = (int) (q * 255);
                break;
        }
        return new Color(r, g, b);
    }
} 