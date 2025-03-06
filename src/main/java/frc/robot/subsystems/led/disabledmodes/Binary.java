package frc.robot.subsystems.led.disabledmodes;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.led.LEDDrawer;
import frc.robot.subsystems.led.Strip;

public class Binary extends LEDDrawer {

  private final Color fg;
  private int time = 0;
  private int wait = 0;

  public Binary(LEDSubsystem susystem, AddressableLED ledStrip, AddressableLEDBuffer buffer, Color fg) {
    super(susystem, ledStrip, buffer);
    this.fg = fg;
  }

  @Override
  public void draw() {
    Color bg = susystem.allianceColor;
    susystem.setColour(susystem.fullStrip, bg);

    int maxValue = (1 << susystem.leftStrip.numLEDs) - 1; // Max binary number for LED count
    int displayValue = time % (maxValue + 1); // Wraparound when reaching max value

    for (Strip strip : susystem.strips) {
      for (int i = 0; i < strip.numLEDs; i++) {
        boolean isOne = ((displayValue >> i) & 1) == 1; // Check if bit is 1
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
