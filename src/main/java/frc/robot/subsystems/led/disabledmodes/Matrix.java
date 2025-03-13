package frc.robot.subsystems.led.disabledmodes;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.led.LEDDrawer;
import frc.robot.subsystems.led.Strip;

public class Matrix extends LEDDrawer {
    private static final int NUM_DROPS = 5;
    private static final double DROP_SPEED = 0.1;
    private static final double FADE_SPEED = 0.05;
    private static final double BRIGHTNESS = 0.4;
    
    private class Drop {
        double position;
        double speed;
        double brightness;
        boolean active;
        
        Drop() {
            reset();
        }
        
        void reset() {
            position = Math.random() * 2 - 1; // Start above the strip
            speed = DROP_SPEED + Math.random() * 0.05;
            brightness = 0;
            active = true;
        }
        
        void update() {
            if (!active) return;
            
            position += speed;
            if (position > 1.0) {
                active = false;
                return;
            }
            
            // Fade in/out based on position
            if (position < 0) {
                brightness = Math.max(0, 1 + position);
            } else if (position > 0.8) {
                brightness = Math.max(0, 1 - (position - 0.8) * 5);
            } else {
                brightness = 1;
            }
        }
    }
    
    private Drop[] drops;
    
    public Matrix(LEDSubsystem subsystem, AddressableLED ledStrip, AddressableLEDBuffer buffer) {
        super(subsystem, ledStrip, buffer);
        drops = new Drop[NUM_DROPS];
        for (int i = 0; i < NUM_DROPS; i++) {
            drops[i] = new Drop();
        }
    }

    @Override
    public void draw() {
        // Set background to very dark green
        susystem.setColour(susystem.fullStrip, new Color(0, 5, 0));
        
        // Update all drops
        for (Drop drop : drops) {
            drop.update();
            if (!drop.active && Math.random() < 0.1) {
                drop.reset();
            }
        }
        
        // Draw drops on each strip
        for (Strip strip : susystem.strips) {
            for (int i = 0; i < strip.numLEDs; i++) {
                double ledPosition = (double) i / strip.numLEDs;
                double maxBrightness = 0;
                
                // Check each drop's influence on this LED
                for (Drop drop : drops) {
                    if (!drop.active) continue;
                    
                    double distance = Math.abs(ledPosition - drop.position);
                    if (distance < 0.1) { // Width of each drop
                        double dropBrightness = drop.brightness * (1 - distance * 10);
                        maxBrightness = Math.max(maxBrightness, dropBrightness);
                    }
                }
                
                // Create the matrix green color
                Color matrixColor = new Color(
                    (int)(255 * maxBrightness * BRIGHTNESS * 0.1),
                    (int)(255 * maxBrightness * BRIGHTNESS),
                    (int)(255 * maxBrightness * BRIGHTNESS * 0.1)
                );
                
                susystem.safeSetLED(strip.start + strip.direction * i, matrixColor);
            }
        }
    }

    @Override
    public int sleepInterval() {
        return 20;
    }
} 