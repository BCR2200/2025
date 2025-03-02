package frc.robot.subsystems.led.disabledmodes;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.led.LEDDrawer;
import frc.robot.subsystems.led.Strip;

public class Rise extends LEDDrawer {

  private final Color fg;
  private int stripIndex = 0;

  public Rise(LEDSubsystem susystem, AddressableLED ledStrip, AddressableLEDBuffer buffer,
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
      susystem.safeSetLED(strip.start + strip.direction * stripIndex, fg);
    }
    stripIndex = (stripIndex + 1) % susystem.strips[0].numLEDs; //modulo thingy, ask hugo if confused
  }

  @Override
  public int sleepInterval() {
    return 30;
  }

}
