package frc.robot.subsystems.led.disabledmodes;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.led.LEDDrawer;
import frc.robot.subsystems.led.Strip;
// NOT DONE
public class Tetris extends LEDDrawer {

  private final Color fg;
  private int stripIndex = 0;
  int placed = 0;

  public Tetris(LEDSubsystem susystem, AddressableLED ledStrip, AddressableLEDBuffer buffer,
              Color fg
  ) {
    super(susystem, ledStrip, buffer);
    this.fg = fg;
  }

  @Override
  public void draw() {
    Color bg = susystem.allianceColor;
    susystem.setColour(susystem.fullStrip, bg);

    for (Strip strip : susystem.strips) {
      susystem.safeSetLED(strip.end - strip.direction * stripIndex, fg);
      for(int i = 0; i < placed; ++i) {
        susystem.safeSetLED(strip.start + strip.direction + 1, fg);
      }
    }

    if(stripIndex < (susystem.strips[0].numLEDs - 1 - placed)){
      stripIndex++; 
    }
    
    if(stripIndex == susystem.strips[0].numLEDs - placed){
      stripIndex = 0;
      placed++; // next one
    }
  }

  @Override
  public int sleepInterval() {
    return 30;
  }

}
