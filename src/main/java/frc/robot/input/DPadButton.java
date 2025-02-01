package frc.robot.input;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DPadButton implements BooleanSupplier {
    public enum DPad {
        Up, Down, Left, Right;

        public Supplier<Boolean> getMethod(XboxController controller) {
            switch (this) {
                case Up:
                case Down:
                case Left: 
                case Right: 
                default: return () -> false;
            }
        }
    }
    XboxController hid;
    public DPad btn;
    double threshold;
    public DPadButton(CommandXboxController controller, DPad btn) {
        this(controller.getHID(), btn);
    }

    public DPadButton(XboxController controller, DPad btn) {
        hid = controller;
        this.btn = btn;
        hid.getPOV();
    }

    @Override 
    public boolean getAsBoolean() {
        int pov = hid.getPOV();
        if (pov == -1) return false;
        switch (btn) {
            case Up: return (pov <= 45) || (pov >= 315);
            case Down: return (135 <= pov && pov <= 225);
            case Left: return (225 <= pov && pov <= 315);
            case Right: return (45 <= pov && pov <= 135);
            default: return false;
        }
    }

    public Trigger trigger() {
        return new Trigger(this);
    }
}
