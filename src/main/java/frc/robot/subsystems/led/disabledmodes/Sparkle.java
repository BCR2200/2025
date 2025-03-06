package frc.robot.subsystems.led.disabledmodes;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.led.LEDDrawer;
import frc.robot.subsystems.led.Strip;

public class Sparkle extends LEDDrawer {
  private int[] sparkleBrightness = { 0, 0 };
  private int[] sparklePosition = { 0, 0 };
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

  public Sparkle(LEDSubsystem subsystem, AddressableLED ledStrip, AddressableLEDBuffer buffer) {
    super(subsystem, ledStrip, buffer);
  }

  @Override
  public void draw() {
        susystem.setColour(susystem.fullStrip, Color.kBlack);
        for (int i = 0; i < susystem.strips.length; i++) {
            if (sparkleBrightness[i] == 0) {
                sparklePosition[i] = (int) (Math.random() * 30);
            }
            int baseInd = strips[i].start + strips[i].direction * sparklePosition[i];
            if (susystem.allianceColor == susystem.BetterRed) {
                susystem.safeSetLED(baseInd, new Color(sparkleBrightness[i], 0, 0));
                if (indexInRange(baseInd + 1, strips[i])) {
                  susystem.safeSetLED(baseInd + 1, new Color((int) (sparkleBrightness[i] * 0.1), 0, 0));
                }
                if (indexInRange(baseInd - 1, strips[i])) {
                    susystem.safeSetLED(baseInd - 1, new Color((int) (sparkleBrightness[i] * 0.1), 0, 0));
                }
            } else {
                susystem.safeSetLED(baseInd, new Color(0, 0, sparkleBrightness[i]));
                if (indexInRange(baseInd + 1, strips[i])) {
                    susystem.safeSetLED(baseInd + 1, new Color(0, 0, (int) (sparkleBrightness[i] * 0.1)));
                }
                if (indexInRange(baseInd - 1, strips[i])) {
                    susystem. safeSetLED(baseInd - 1, new Color(0, 0, (int) (sparkleBrightness[i] * 0.1)));
                }
            }

            if (sparkleBrightness[i] >= 75) {
                sparkleDirection[i] = false;
            } else if (sparkleBrightness[i] <= 0) {
                sparkleDirection[i] = true;
            }
            if (sparkleDirection[i]) {
                sparkleBrightness[i]++;
            } else {
                sparkleBrightness[i]--;
            }
        }
    }
  @Override
  public int sleepInterval() {
    return 10;
  }
}
