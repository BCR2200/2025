package frc.robot.subsystems.led.disabledmodes;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.led.LEDDrawer;
import frc.robot.subsystems.led.Strip;

public class Fill extends LEDDrawer {

  private int stripIndex = 0;
  private boolean goingUp = true;
  Color bg = susystem.allianceColor;
  Color fg = new Color(50,50,35);
  Color default_fg = new Color(50,50,35); 
  Color temp;
  private int goingDirection;


  public Fill(LEDSubsystem susystem, AddressableLED ledStrip, AddressableLEDBuffer buffer) {
    super(susystem, ledStrip, buffer);
  }

  @Override
  public void draw() {   

    if (goingUp) {
      stripIndex++;
      susystem.setColour(susystem.fullStrip, bg);
      if (stripIndex >= susystem.strips[0].numLEDs-1) {
        goingUp = false;
        goingDirection = 1;
      }
    } else {
      stripIndex--;

      if (stripIndex <= 0) {
        goingUp = true;
        goingDirection = -1;
        temp = fg;
        fg = bg;
        bg = temp;
        if (fg.equals(default_fg)){
          bg = susystem.allianceColor; // works lol
        }
      }
    }

    for (Strip strip : susystem.strips) {
      susystem.safeSetLED(strip.start + strip.direction * stripIndex, fg);
      susystem.safeSetLED(strip.start + strip.direction * stripIndex - goingDirection, fg);
    }
  }

  @Override
  public int sleepInterval() {
    return 40;
  }

}
