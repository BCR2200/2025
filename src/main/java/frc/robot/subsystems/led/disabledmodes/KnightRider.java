package frc.robot.subsystems.led.disabledmodes;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.led.LEDDrawer;
import frc.robot.subsystems.led.Strip;

public class KnightRider extends LEDDrawer {
    private double position = 0;
    private boolean goingRight = true;
    private static final double SPEED = 0.05;
    private static final double TAIL_LENGTH = 0.15; // Length of the trailing effect (0-1)
    private static final double MAX_BRIGHTNESS = 0.8;

    public KnightRider(LEDSubsystem subsystem, AddressableLED ledStrip, AddressableLEDBuffer buffer) {
        super(subsystem, ledStrip, buffer);
    }

    @Override
    public void draw() {
        // Set background to alliance color
        susystem.setColour(susystem.fullStrip, susystem.allianceColor);

        // Update position
        if (goingRight) {
            position += SPEED;
            if (position >= 1.0) {
                position = 1.0;
                goingRight = false;
            }
        } else {
            position -= SPEED;
            if (position <= 0.0) {
                position = 0.0;
                goingRight = true;
            }
        }

        // Draw scanning effect on each strip
        for (Strip strip : susystem.strips) {
            for (int i = 0; i < strip.numLEDs; i++) {
                // Calculate normalized position (0-1)
                double ledPosition = (double) i / strip.numLEDs;
                
                // Calculate distance from scanner position
                double distance = Math.abs(ledPosition - position);
                
                // Calculate brightness based on distance and direction
                double brightness = 0;
                if (distance < TAIL_LENGTH) {
                    // Create smooth falloff
                    brightness = Math.pow(1 - (distance / TAIL_LENGTH), 2);
                }
                
                // Create the scanner color (red for red alliance, blue for blue alliance)
                Color scannerColor;
                if (susystem.allianceColor.equals(susystem.BetterRed)) {
                    scannerColor = new Color(
                        (int)(255 * brightness * MAX_BRIGHTNESS),
                        (int)(255 * brightness * MAX_BRIGHTNESS * 0.1),
                        (int)(255 * brightness * MAX_BRIGHTNESS * 0.1)
                    );
                } else {
                    scannerColor = new Color(
                        (int)(255 * brightness * MAX_BRIGHTNESS * 0.1),
                        (int)(255 * brightness * MAX_BRIGHTNESS * 0.1),
                        (int)(255 * brightness * MAX_BRIGHTNESS)
                    );
                }
                
                susystem.safeSetLED(strip.start + strip.direction * i, scannerColor);
            }
        }
    }

    @Override
    public int sleepInterval() {
        return 20;
    }
} 