package frc.robot.subsystems.led.disabledmodes;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.ReaLEDSubsystem;
import frc.robot.subsystems.led.LEDDrawer;

public class Full extends LEDDrawer {

  private final Color clr;
  private Color colour;

  public Full(ReaLEDSubsystem susystem, AddressableLED ledStrip, AddressableLEDBuffer buffer,
      Color clr) {
    super(susystem, ledStrip, buffer);
    this.clr = clr;
  }

  public Full(ReaLEDSubsystem susystem, AddressableLED ledStrip, AddressableLEDBuffer buffer) {
    super(susystem, ledStrip, buffer);
    clr = null;
  }

  @Override
  public void draw() {
    if (clr == null) {
      colour = susystem.allianceColor;
    } else {
      colour = clr;
    }
    susystem.setColour(susystem.fullStrip, colour);
  }

  @Override
  public int sleepInterval() {
    return 100;
  }

}
