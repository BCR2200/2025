package frc.robot.subsystems.led.disabledmodes;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.led.LEDDrawer;
import frc.robot.subsystems.led.Strip;

// NOT DONE
public class Tetris extends LEDDrawer {

  private final Color fg;
  private int stripIndex = 0;
  int placed = 0;
  boolean slidingTime = false;

  public Tetris(LEDSubsystem susystem, AddressableLED ledStrip, AddressableLEDBuffer buffer,
      Color fg) {
    super(susystem, ledStrip, buffer);
    this.fg = fg;
  }

  @Override
  public void draw() {
    Color bg = susystem.allianceColor;

    SmartDashboard.putNumber("stripIndex", stripIndex);
    SmartDashboard.putNumber("placed", placed);
    SmartDashboard.putNumber("numleds", susystem.leftStrip.numLEDs);
    SmartDashboard.putBoolean("slide", slidingTime);

    if (slidingTime) {
      susystem.setColour(susystem.fullStrip, fg);
    } else {
      susystem.setColour(susystem.fullStrip, bg);
    }

    for (Strip strip : susystem.strips) {
      if (slidingTime) {
        // susystem.safeSetLED(strip.start + strip.direction * stripIndex, bg);

        for (int i = 0; i < placed; ++i) {
          susystem.safeSetLED(strip.end - strip.direction * i, bg);
        }

      } else {
        susystem.safeSetLED(strip.end - strip.direction * stripIndex, fg);

        for (int i = 0; i < placed; ++i) {
          susystem.safeSetLED(strip.start + strip.direction * i, fg);
        }
        // if (stripIndex < placed) {
        // susystem.safeSetLED(strip.start + strip.direction * stripIndex, fg);
        // }

      }
    }
    
    if (slidingTime) {
      if (stripIndex < (susystem.leftStrip.numLEDs - 1)) {
        placed++;
      }

      if (placed == susystem.leftStrip.numLEDs) {
        slidingTime = false;
        stripIndex = 0;
        placed = 0;
      }
    } else {
      if (stripIndex < (susystem.leftStrip.numLEDs - 1 - placed)) {
        stripIndex++;
      }

      if (stripIndex == susystem.leftStrip.numLEDs - 1 - placed) {
        stripIndex = 0;
        placed++; // next one
      }

      if (placed == susystem.leftStrip.numLEDs - 1) {
        slidingTime = true;
        placed = 0;
        stripIndex = 0;
      }
    }

  }

  @Override
  public int sleepInterval() {
    return 60;
  }

}
