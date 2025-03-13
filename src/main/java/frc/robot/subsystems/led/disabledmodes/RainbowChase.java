package frc.robot.subsystems.led.disabledmodes;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.led.LEDDrawer;
import frc.robot.subsystems.led.Strip;

public class RainbowChase extends LEDDrawer {
    private static final int NUM_WAVES = 3;
    private static final double WAVE_SPEED = 0.02;
    private static final double WAVE_SPACING = 0.33; // Space between waves
    private static final double WAVE_WIDTH = 0.2; // Width of each wave
    private static final double BRIGHTNESS = 0.7;
    
    private class Wave {
        double position;
        double hue;
        double speed;
        
        Wave(double startPos, double startHue, double speed) {
            this.position = startPos;
            this.hue = startHue;
            this.speed = speed;
        }
        
        void update() {
            position = (position + speed) % 1.0;
            hue = (hue + 0.001) % 1.0; // Slowly change hue
        }
        
        double getBrightness(double ledPos) {
            double distance = Math.abs(ledPos - position);
            if (distance > WAVE_WIDTH) return 0;
            return Math.pow(1 - (distance / WAVE_WIDTH), 2);
        }
    }
    
    private Wave[] waves;
    
    public RainbowChase(LEDSubsystem subsystem, AddressableLED ledStrip, AddressableLEDBuffer buffer) {
        super(subsystem, ledStrip, buffer);
        waves = new Wave[NUM_WAVES];
        
        // Initialize waves with different starting positions and hues
        for (int i = 0; i < NUM_WAVES; i++) {
            waves[i] = new Wave(
                i * WAVE_SPACING, // Stagger starting positions
                i / (double)NUM_WAVES, // Stagger starting hues
                WAVE_SPEED * (1 + i * 0.1) // Slightly different speeds
            );
        }
    }

    @Override
    public void draw() {
        // Set background to alliance color
        susystem.setColour(susystem.fullStrip, susystem.allianceColor);
        
        // Update all waves
        for (Wave wave : waves) {
            wave.update();
        }
        
        // Draw waves on each strip
        for (Strip strip : susystem.strips) {
            for (int i = 0; i < strip.numLEDs; i++) {
                double ledPosition = (double) i / strip.numLEDs;
                double maxBrightness = 0;
                double finalHue = 0;
                
                // Check each wave's influence on this LED
                for (Wave wave : waves) {
                    double brightness = wave.getBrightness(ledPosition);
                    if (brightness > maxBrightness) {
                        maxBrightness = brightness;
                        finalHue = wave.hue;
                    }
                }
                
                // Convert HSV to RGB
                Color chaseColor = hsvToRgb(finalHue, 1.0, maxBrightness * BRIGHTNESS);
                susystem.safeSetLED(strip.start + strip.direction * i, chaseColor);
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