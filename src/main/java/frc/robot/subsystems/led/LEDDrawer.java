package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.subsystems.LEDSubsystem;

public abstract class LEDDrawer {
    protected final LEDSubsystem susystem;
    protected final AddressableLED ledStrip;
    protected final AddressableLEDBuffer buffer;

    public LEDDrawer(LEDSubsystem susystem, AddressableLED ledStrip, AddressableLEDBuffer buffer) {
        this.susystem = susystem;
        this.ledStrip = ledStrip;
        this.buffer = buffer;
    }

    public abstract void draw();

    public abstract int sleepInterval();
}
