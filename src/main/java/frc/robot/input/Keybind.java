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
      return switch (this) {
        case A -> controller::getAButton;
        case B -> controller::getBButton;
        case X -> controller::getXButton;
        case Y -> controller::getYButton;
        case LeftBumper -> controller::getLeftBumperButton;
        case RightBumper -> controller::getRightBumperButton;
        case LeftStick -> controller::getLeftStickButton;
        case RightStick -> controller::getRightStickButton;
        case Start -> controller::getStartButton;
        case Select -> controller::getBackButton;
      };
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
