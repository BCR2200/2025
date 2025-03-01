package frc.robot.subsystems.led.disabledmodes;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.led.LEDDrawer;

public class Sink extends LEDDrawer {

    private final Color fg;

    public Sink(LEDSubsystem susystem, AddressableLED ledStrip, AddressableLEDBuffer buffer,
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
        susystem.safeSetLED(susystem.strips[i].end + -susystem.strips[i].direction * susystem.stripIndex, fg);
      }
      susystem.stripIndex = (susystem.stripIndex+1) % susystem.strips[0].numLEDs; //modulo thingy, ask hugo if confused
    }

	@Override
	public int sleepInterval() {
        return 30;
	}

}
