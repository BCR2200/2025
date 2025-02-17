package frc.robot.input;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DPadButton implements BooleanSupplier {
    public enum DPad {
        // Allow two DPad buttons to be pressed and still trigger.
        // Ex: Up and Right pressed, POV -> 45, Up & Right both triggered.
        Up, Down, Left, Right,
        // Only exactly that DPad direction pressed will be triggered
        // Ex: Up and Right pressed, POV -> 45, Neither Up nor Right triggered.
        // Ex: Up pressed, POV -> 0, Up triggered.
        OnlyUp, OnlyDown, OnlyLeft, OnlyRight;

        public boolean isTriggered(int pov) {
          return switch (this) {
            case Up -> (pov >= 315) || (pov <= 45); // Different due to 360 wraparound
            case Down -> (135 <= pov && pov <= 225);
            case Left -> (225 <= pov && pov <= 315);
            case Right -> (45 <= pov && pov <= 135);
            case OnlyUp -> pov == 0;
            case OnlyDown -> pov == 180;
            case OnlyLeft -> pov == 270;
            case OnlyRight -> pov == 90;
          };
        }
    }
    XboxController hid;
    public DPad btn;
    public DPadButton(CommandXboxController controller, DPad btn) {
        this(controller.getHID(), btn);
    }
    public DPadButton(XboxController controller, DPad btn) {
      hid = controller;
      this.btn = btn;
    }

    /*
    public DPadButton(POVSupplier controller, DPad btn) {
        hid = controller;
        this.btn = btn;
    }

     */

    @Override 
    public boolean getAsBoolean() {
        int pov = hid.getPOV();
        if (pov == -1) return false;

        return btn.isTriggered(pov);
    }

    public Trigger trigger() {
        return new Trigger(this);
    }
}
