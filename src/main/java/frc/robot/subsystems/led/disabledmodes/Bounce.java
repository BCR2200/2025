package frc.robot.subsystems.led.disabledmodes;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.led.LEDDrawer;
import frc.robot.subsystems.led.Strip;

public class Bounce extends LEDDrawer {

  private final Color fg;
  private int stripIndex = 0;
  private boolean goingUp = true;

  public Bounce(LEDSubsystem susystem, AddressableLED ledStrip, AddressableLEDBuffer buffer,
      Color fg) {
    super(susystem, ledStrip, buffer);
    this.fg = fg;
  }

  @Override
  public void draw() {   
    Color bg = susystem.allianceColor; 
    susystem.setColour(susystem.fullStrip, bg);
    
    for (Strip strip : susystem.strips) {
      susystem.safeSetLED(strip.start + strip.direction * stripIndex, fg);
    }
    
    if (goingUp) {
      stripIndex++;
      if (stripIndex >= susystem.strips[0].numLEDs-1) {
        goingUp = false;
      }
    } else {
      stripIndex--;
      if (stripIndex <= 0) {
        goingUp = true;
      }
    }
  }

  @Override
  public int sleepInterval() {
    return 30;
  }

}
