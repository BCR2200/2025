package frc.robot.subsystems.led.disabledmodes;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.ReaLEDSubsystem;
import frc.robot.subsystems.led.LEDDrawer;

public class Rise extends LEDDrawer {

    private final Color fg;

    public Rise(ReaLEDSubsystem susystem, AddressableLED ledStrip, AddressableLEDBuffer buffer,
        Color fg
    ) {
        super(susystem, ledStrip, buffer);
        this.fg = fg;
    }

    @Override
    public void draw() {
        Color bg = susystem.allianceColor;
        susystem.setColour(susystem.fullStrip, bg);
      for (int i = 0; i < susystem.strips.length; i++) {
        susystem.safeSetLED(susystem.strips[i].start + susystem.strips[i].direction * susystem.stripIndex, fg);
      }
      susystem.stripIndex++;
      if (susystem.stripIndex == susystem.strips[0].numLEDs) {
        susystem.stripIndex = 0;
      }
    }

	@Override
	public int sleepInterval() {
        return 30;
	}

}
