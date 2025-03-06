package frc.robot.subsystems.led.disabledmodes;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.led.LEDDrawer;

public class Crackle extends LEDDrawer {
  private Color bg = susystem.allianceColor;
  private Color fg = susystem.BetterWhite;

  public Crackle(LEDSubsystem subsystem, AddressableLED ledStrip, AddressableLEDBuffer buffer) {
    super(subsystem, ledStrip, buffer);
  }

  @Override
  public void draw() {
    susystem.setColour(susystem.fullStrip, bg);
    susystem.safeSetLED(susystem.leftStrip.start + susystem.leftStrip.direction * (int) (Math.random() * 30), fg);
    susystem.safeSetLED(susystem.rightStrip.start + susystem.rightStrip.direction * (int) (Math.random() * 30), fg);
  }
  @Override
  public int sleepInterval() {
    return 60;
  }
}
