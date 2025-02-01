package frc.robot.input;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.input.DPadButton.DPad;

public class AnalogTrigger implements BooleanSupplier {
    public enum Axis {
        LX, LY, RX, RY, LT, RT;

        public DoubleSupplier getMethod(XboxController controller) {
            switch (this) {
                case LX: return controller::getLeftX;
                case LY: return controller::getLeftY;
                case RX: return controller::getRightX;
                case RY: return controller::getRightY;
                case LT: return controller::getLeftTriggerAxis;
                case RT: return controller::getRightTriggerAxis;
                default: return () -> 0;
            }
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
