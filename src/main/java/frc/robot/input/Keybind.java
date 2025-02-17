package frc.robot.input;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Keybind implements BooleanSupplier {
    public enum Button {
        A, B, X, Y, Start, Select,
        LeftBumper, RightBumper,
        LeftStick, RightStick;

        public Supplier<Boolean> getMethod(XboxController controller) {
            switch (this) {
                case A: return controller::getAButton;
                case B: return controller::getBButton;
                case X: return controller::getXButton;
                case Y: return controller::getYButton;
                case LeftBumper: return controller::getLeftBumperButton;
                case RightBumper: return controller::getRightBumperButton;
                case LeftStick: return controller::getLeftStickButton;
                case RightStick: return controller::getRightStickButton;
                case Start: return controller::getStartButton;
                case Select: return controller::getBackButton;
                default: return () -> false;
            }
        }
    }
    XboxController hid;
    public Button button;
    public Keybind(CommandXboxController controller, Button button) {
        this(controller.getHID(), button);
    }
    public Keybind(XboxController controller, Button button) {
        hid = controller;
        this.button = button;
    }
    @Override 
    public boolean getAsBoolean() {
        return button.getMethod(hid).get();
    }

    public Trigger trigger() {
        return new Trigger(this);
    }
}
