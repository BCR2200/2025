package frc.robot.subsystems.led.disabledmodes;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.led.LEDDrawer;
import frc.robot.subsystems.led.Strip;

public class Checkerboard extends LEDDrawer {
    private static final int SQUARE_SIZE = 3; // Number of LEDs per square
    private static final double BRIGHTNESS = 0.8; // Brightness of the alliance color
    
    public Checkerboard(LEDSubsystem subsystem, AddressableLED ledStrip, AddressableLEDBuffer buffer) {
        super(subsystem, ledStrip, buffer);
    }

    @Override
    public void draw() {
        // Draw the checkerboard pattern on each strip
        for (Strip strip : susystem.strips) {
            for (int i = 0; i < strip.numLEDs; i++) {
                // Calculate which square this LED belongs to
                int squareIndex = i / SQUARE_SIZE;
                // Calculate if this square should be colored or black
                boolean isColored = (squareIndex % 2) == 0;
                
                Color ledColor;
                if (isColored) {
                    // Use alliance color with adjusted brightness
                    ledColor = new Color(
                        (int)(susystem.allianceColor.red * BRIGHTNESS * 255),
                        (int)(susystem.allianceColor.green * BRIGHTNESS * 255),
                        (int)(susystem.allianceColor.blue * BRIGHTNESS * 255)
                    );
                } else {
                    ledColor = Color.kBlack;
                }
                
                susystem.safeSetLED(strip.start + strip.direction * i, ledColor);
            }
        }
    }

    @Override
    public int sleepInterval() {
        return 20;
    }
} 