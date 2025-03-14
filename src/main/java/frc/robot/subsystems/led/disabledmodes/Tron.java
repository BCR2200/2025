package frc.robot.subsystems.led.disabledmodes;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.led.LEDDrawer;
import frc.robot.subsystems.led.Strip;

public class Tron extends LEDDrawer {
    private static final double SPEED = 0.02;
    private static final double TRAIL_LENGTH = 0.3; // Length of the trail as a percentage of the strip
    private static final double HEAD_BRIGHTNESS = 1.0;
    private static final double TRAIL_BRIGHTNESS = 0.6;
    private static final Color TRON_BLUE = new Color(0, 0.5, 1.0); // Bright cyan color
    
    private double position = 0;
    private boolean direction = true; // true = moving right, false = moving left
    
    public Tron(LEDSubsystem subsystem, AddressableLED ledStrip, AddressableLEDBuffer buffer) {
        super(subsystem, ledStrip, buffer);
    }

    @Override
    public void draw() {
        // Set background to very dark blue
        susystem.setColour(susystem.fullStrip, new Color(0, 0, 5));
        
        // Update position
        if (direction) {
            position += SPEED;
            if (position >= 1.0) {
                direction = false;
            }
        } else {
            position -= SPEED;
            if (position <= 0.0) {
                direction = true;
            }
        }
        
        // Draw the effect on each strip
        for (Strip strip : susystem.strips) {
            for (int i = 0; i < strip.numLEDs; i++) {
                double ledPosition = (double) i / strip.numLEDs;
                double brightness = 0;
                
                // Calculate distance from the current position
                double distance = Math.abs(ledPosition - position);
                
                // Create the trail effect
                if (distance < TRAIL_LENGTH) {
                    // Bright head
                    if (distance < 0.05) {
                        brightness = HEAD_BRIGHTNESS;
                    } else {
                        // Fading trail
                        brightness = TRAIL_BRIGHTNESS * (1 - (distance / TRAIL_LENGTH));
                    }
                }
                
                // Create the Tron color with the calculated brightness
                Color tronColor = new Color(
                    (int)(TRON_BLUE.red * brightness * 255),
                    (int)(TRON_BLUE.green * brightness * 255),
                    (int)(TRON_BLUE.blue * brightness * 255)
                );
                
                susystem.safeSetLED(strip.start + strip.direction * i, tronColor);
            }
        }
    }

    @Override
    public int sleepInterval() {
        return 20;
    }
} 