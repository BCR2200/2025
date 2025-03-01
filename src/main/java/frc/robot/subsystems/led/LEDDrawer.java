package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.subsystems.ReaLEDSubsystem;

public abstract class LEDDrawer {
    protected final ReaLEDSubsystem susystem;
    protected final AddressableLED ledStrip;
    protected final AddressableLEDBuffer buffer;

    public LEDDrawer(ReaLEDSubsystem susystem, AddressableLED ledStrip, AddressableLEDBuffer buffer) {
        this.susystem = susystem;
        this.ledStrip = ledStrip;
        this.buffer = buffer;
    }

    public abstract void draw();

    public abstract int sleepInterval();
}
