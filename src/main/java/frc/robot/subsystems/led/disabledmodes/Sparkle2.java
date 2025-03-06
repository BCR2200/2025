package frc.robot.subsystems.led.disabledmodes;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.led.LEDDrawer;
import frc.robot.subsystems.led.Strip;

public class Sparkle2 extends LEDDrawer {
  private int[] sparkleBrightness = { 0, 0 };
  private int[] sparklePosition = { 0, 0, };
  private boolean[] sparkleDirection = { true, false };
  private Strip[] strips = susystem.strips;

  public boolean indexInRange(int val, Strip strip) {
    int min;
    int max;
    if (strip.direction == 1) {
        min = strip.start;
        max = strip.end;
    } else {
        max = strip.start;
        min = strip.end;
    }
    return val >= min && val <= max;
  }

  public Sparkle2(LEDSubsystem subsystem, AddressableLED ledStrip, AddressableLEDBuffer buffer) {
    super(subsystem, ledStrip, buffer);
  }

  @Override
  public void draw() {
        for (int i = 0; i < strips.length; i++) {
            if (sparkleBrightness[i] <= 0) {
                sparklePosition[i] = (int) (Math.random() * 30);
            }
            int baseInd = strips[i].start + strips[i].direction * sparklePosition[i];
            if (susystem.allianceColor == susystem.BetterRed) {

                susystem.setColour(strips[i], new Color(20, 0, 0));
                susystem.safeSetLED(baseInd, new Color(sparkleBrightness[i], sparkleBrightness[i], sparkleBrightness[i]));
                Color ColorRed = new Color((int) (sparkleBrightness[i] * 0.3), (int) (sparkleBrightness[i] * 0.1), (int) (sparkleBrightness[i] * 0.1)); //This is out here because it is used twice i don't waana make a new color twice, so i stored it in a variable
                
                if (indexInRange(baseInd + 1, strips[i])) {
                    susystem.safeSetLED(baseInd + 1, ColorRed);
                }
                if (indexInRange(baseInd - 1, strips[i])) {
                    susystem.safeSetLED(baseInd - 1, ColorRed);
                }
            } else {
                susystem.setColour(strips[i], new Color(0, 0, 20));
                susystem.safeSetLED(baseInd, new Color(sparkleBrightness[i], sparkleBrightness[i], sparkleBrightness[i]));
                Color ColorBlue = new Color((int) (sparkleBrightness[i] * 0.1), (int) (sparkleBrightness[i] * 0.1), (int) (sparkleBrightness[i] * 0.3)); //This is out here because it is used twice i don't waana make a new color twice, so i stored it in a variable


                if (indexInRange(baseInd + 1, strips[i])) {
                    susystem.safeSetLED(baseInd + 1, ColorBlue);
                }
                if (indexInRange(baseInd - 1, strips[i])) {
                    susystem.safeSetLED(baseInd - 1, ColorBlue);
                }
            }
            if (sparkleBrightness[i] >= 75) {
                sparkleDirection[i] = false;
            } else if (sparkleBrightness[i] <= 0) {
                sparkleDirection[i] = true;
            }
            if (sparkleDirection[i]) {
                sparkleBrightness[i] += 4;
            } else {
                sparkleBrightness[i] -= 4;
            }
        }
    }
  @Override
  public int sleepInterval() {
    return 20;
  }
}
