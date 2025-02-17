package frc.robot.input;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AnalogTrigger implements BooleanSupplier {
  public enum Axis {
    LX, LY, RX, RY, LT, RT;

    public DoubleSupplier getMethod(XboxController controller) {
      return switch (this) {
        case LX -> controller::getLeftX;
        case LY -> controller::getLeftY;
        case RX -> controller::getRightX;
        case RY -> controller::getRightY;
        case LT -> controller::getLeftTriggerAxis;
        case RT -> controller::getRightTriggerAxis;
      };
    }
  }

  XboxController hid;
  public Axis axis;
  double threshold;

  public AnalogTrigger(CommandXboxController controller, Axis axis, double threshold) {
    this(controller.getHID(), axis, threshold);
  }

  public AnalogTrigger(XboxController controller, Axis axis, double threshold) {
    hid = controller;
    this.axis = axis;
    this.threshold = threshold;
  }

  @Override
  public boolean getAsBoolean() {
    return axis.getMethod(hid).getAsDouble() >= threshold;
  }

  public Trigger trigger() {
    return new Trigger(this);
  }
}
