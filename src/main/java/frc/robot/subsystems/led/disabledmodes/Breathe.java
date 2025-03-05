package frc.robot.subsystems.led.disabledmodes;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.led.LEDDrawer;

public class Breathe extends LEDDrawer {

  double stripIndex = 0;
  boolean breatheDirection = true;

  public Breathe(LEDSubsystem subsystem, AddressableLED ledStrip, AddressableLEDBuffer buffer) {
    super(subsystem, ledStrip, buffer);
  }

  @Override
  public void draw() {
    if (susystem.allianceColor.equals(susystem.BetterRed)) {
      susystem.setColour(susystem.fullStrip, new Color(stripIndex/100.0, 0, 0));
    } else {
      susystem.setColour(susystem.fullStrip, new Color(0, 0, stripIndex/100.0));
    }
    if (stripIndex >= 75) {
      breatheDirection = false;
    } else if (stripIndex <= 0) {
      breatheDirection = true;
    }
    if (breatheDirection) {
      stripIndex++;
    } else {
      stripIndex--;
    }
  }

  @Override
  public int sleepInterval() {
    return 30;
  }
}
