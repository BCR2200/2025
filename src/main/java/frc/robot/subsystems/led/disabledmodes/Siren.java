package frc.robot.subsystems.led.disabledmodes;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.led.LEDDrawer;

public class Siren extends LEDDrawer {
  private Color color1 = susystem.BetterRed;
  private Color color2 = susystem.BetterBlue;
  private Color colorA;
  private Color colorB;
  private boolean sirenState = true;

  public Siren(LEDSubsystem subsystem, AddressableLED ledStrip, AddressableLEDBuffer buffer) {
    super(subsystem, ledStrip, buffer);
  }

  @Override
  public void draw() {
    if (sirenState) {
      colorA = color2;
      colorB = color1;
    } else {
      colorA = color1;
      colorB = color2;
    }
    for (var strip : susystem.halfTopStrips) {
      susystem.setColour(strip, colorA);
    }
    for (var strip : susystem.halfBotStrips) {
      susystem.setColour(strip, colorB);
    }
    sirenState = !sirenState;
  }

  @Override
  public int sleepInterval() {
    return 100;
  }
}
