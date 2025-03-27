package frc.robot.subsystems.led.disabledmodes;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.led.LEDDrawer;
import frc.robot.subsystems.led.Strip;

public class DoubleBinary extends LEDDrawer {

  private final Color fg;
  private int time = 0;
  private int wait = 0;

  public DoubleBinary(LEDSubsystem susystem, AddressableLED ledStrip, AddressableLEDBuffer buffer, Color fg) {
    super(susystem, ledStrip, buffer);
    this.fg = fg;
  }

  @Override
  public void draw() {
    Color bg = susystem.allianceColor;
    susystem.setColour(susystem.fullStrip, bg);

    int maxValue = (1 << susystem.leftTopStrip.numLEDs) - 1; // Max binary number for LED count
    int displayValue = time % (maxValue + 1); // Wraparound when reaching max value

    // Draw on top strips (from top to bottom)
    for (Strip strip : susystem.topLBotRStrips) {
      for (int i = 0; i < strip.numLEDs; i++) {
        // Reverse the bit order for top strips
        boolean isOne = ((displayValue >> (strip.numLEDs - 1 - i)) & 1) == 1;
        susystem.safeSetLED(strip.start + strip.direction * i, isOne ? fg : bg);
      }
    }

    // Draw on bottom strips (from bottom to top)
    for (Strip strip : susystem.topRBotLStrips) {
      for (int i = 0; i < strip.numLEDs; i++) {
        // For right strips, count from outside in
        boolean isOne = ((displayValue >> (strip.numLEDs - 1 - i)) & 1) == 1;
        susystem.safeSetLED(strip.start + strip.direction * i, isOne ? fg : bg);
      }
    }

    if(time < 2200){
      time += 10; // Increment binary number
    } else{
      wait++;
      if(wait > 50){
        wait = 0;
        time = 0;
      }
    }
    SmartDashboard.putNumber("num", time);
  }

  @Override
  public int sleepInterval() {
    return 100;
  }
} 