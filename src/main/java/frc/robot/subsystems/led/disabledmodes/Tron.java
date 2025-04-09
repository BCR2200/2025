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
    private static final Color TRON_RED = new Color(1.0, 0.5, 0); // Bright red color
    
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
                
                Color tronColor;
                // Determine the color of the trail based on the alliance color
                if (susystem.allianceColor == susystem.BetterRed) {
                    tronColor = TRON_RED; 
                } else {
                    tronColor = TRON_BLUE; 
                }

                // Adjust the color brightness based on the calculated brightness value
                tronColor = new Color(
                    (int)(tronColor.red * brightness * 255),   
                    (int)(tronColor.green * brightness * 255), 
                    (int)(tronColor.blue * brightness * 255)  
                );

                // Set the LED at the calculated position on the strip to the adjusted color
                susystem.safeSetLED(strip.start + strip.direction * i, tronColor);
            }
        }
    }

    @Override
    public int sleepInterval() {
        return 20;
    }
}
