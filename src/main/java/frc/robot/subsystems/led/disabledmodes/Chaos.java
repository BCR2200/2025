package frc.robot.subsystems.led.disabledmodes;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.led.LEDDrawer;
import frc.robot.subsystems.led.Strip;

public class Chaos extends LEDDrawer {
    private static final int NUM_PARTICLES = 10;
    private static final double SPEED = 0.1;
    private static final double COLOR_CHANGE_SPEED = 0.05;
    private static final double BRIGHTNESS = 1.0;
    
    private class Particle {
        double position;
        double speed;
        double hue;
        double saturation;
        double value;
        boolean active;
        
        Particle() {
            reset();
        }
        
        void reset() {
            position = Math.random();
            speed = (Math.random() - 0.5) * SPEED;
            hue = Math.random();
            saturation = 1.0;
            value = 1.0;
            active = true;
        }
        
        void update() {
            if (!active) return;
            
            position = (position + speed) % 1.0;
            if (position < 0) position += 1.0;
            
            // Randomly change color
            hue = (hue + (Math.random() - 0.5) * COLOR_CHANGE_SPEED) % 1.0;
            if (hue < 0) hue += 1.0;
            
            // Randomly change speed
            if (Math.random() < 0.1) {
                speed = (Math.random() - 0.5) * SPEED;
            }
        }
    }
    
    private Particle[] particles;
    private double time = 0;
    private double globalHue = 0;
    
    public Chaos(LEDSubsystem subsystem, AddressableLED ledStrip, AddressableLEDBuffer buffer) {
        super(subsystem, ledStrip, buffer);
        particles = new Particle[NUM_PARTICLES];
        for (int i = 0; i < NUM_PARTICLES; i++) {
            particles[i] = new Particle();
        }
    }

    @Override
    public void draw() {
        time += 0.01;
        globalHue = (globalHue + 0.001) % 1.0;
        
        // Update all particles
        for (Particle particle : particles) {
            particle.update();
        }
        
        // Draw the effect on each strip
        for (Strip strip : susystem.strips) {
            for (int i = 0; i < strip.numLEDs; i++) {
                double ledPosition = (double) i / strip.numLEDs;
                double maxBrightness = 0;
                double finalHue = 0;
                
                // Calculate influence from each particle
                for (Particle particle : particles) {
                    if (!particle.active) continue;
                    
                    double distance = Math.abs(ledPosition - particle.position);
                    if (distance < 0.2) { // Influence radius
                        double brightness = (1 - distance * 5) * BRIGHTNESS;
                        if (brightness > maxBrightness) {
                            maxBrightness = brightness;
                            finalHue = particle.hue;
                        }
                    }
                }
                
                // Add some random noise
                if (Math.random() < 0.1) {
                    maxBrightness = Math.min(1.0, maxBrightness + Math.random() * 0.3);
                }
                
                // Create a chaotic color
                Color chaosColor = hsvToRgb(finalHue, 1.0, maxBrightness);
                
                // Add some random color shifting
                if (Math.random() < 0.05) {
                    chaosColor = new Color(
                        (int)(Math.random() * 255),
                        (int)(Math.random() * 255),
                        (int)(Math.random() * 255)
                    );
                }
                
                susystem.safeSetLED(strip.start + strip.direction * i, chaosColor);
            }
        }
    }

    @Override
    public int sleepInterval() {
        return 10; // Very fast updates for maximum chaos
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