package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDMotor {

    final String name;
    boolean initialized = false;

    /*
     * p - output per unit of error in velocity (output/rps)
     * i - output per unit of integrated error in velocity (output/rotation)
     * d - output per unit of error derivative in velocity (output/(rps/s))
     * s - output to overcome static friction (output)
     * v - output per unit of target velocity (output/rps)
     * a - output per unit of target acceleration (output/(rps/s))
     * not used (YET):
     * g - output to overcome gravity (output)
     */
    double p, i, d, s, v, a, g;
    double maxV;
    double maxA;
    double maxJerk;

    // This can be used as both position and velocity target
    double target = 0.0;

    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
    private final DynamicMotionMagicVoltage dynamicMotionMagicVoltage = new DynamicMotionMagicVoltage(0, 0, 0, 0);

    final TalonFXConfiguration talonFXConfigs;
    public final TalonFX motor;

    private PIDMotor(int deviceID, String name, double p, double i, double d, double s, double v, double a, double g, double maxV,
            double maxA, double maxJerk, boolean enableFOC) {
        this.name = name;
        this.p = p;
        this.i = i;
        this.d = d;
        this.s = s;
        this.v = v;
        this.a = a;
        this.g = g;
        this.maxV = maxV;
        this.maxA = maxA;
        this.maxJerk = maxJerk;

        dutyCycleOut.withEnableFOC(true);
        motionMagicVoltage.withEnableFOC(enableFOC);
        dynamicMotionMagicVoltage.withEnableFOC(true);

        motor = new TalonFX(deviceID, "*");
        talonFXConfigs = new TalonFXConfiguration();
    }

    /**
     * Constructs and initializes a new PIDMotor.
     * 
     * @param deviceID The CAN ID for this motor.
     * @param name     The name of this motor.
     * @param p        The initial proportional coefficient for the PID controller.
     * @param i        The initial integral coefficient for the PID controller.
     * @param d        The initial derivative coefficient for the PID controller.
     * @param s        The initial feed-forward coefficient for the motor (may refer
     *                 to a constant like velocity or position feed-forward).
     * @param v        The initial velocity setpoint for the motor.
     * @param a        The initial acceleration setpoint for the motor.
     * @param maxV     The maximum allowable velocity for the motor.
     * @param maxA     The maximum allowable acceleration for the motor.
     * @param maxJerk  The maximum allowable jerk (rate of change of acceleration)
     *                 for the motor.
     * @return The initialized PIDMotor instance.
     */
    public static PIDMotor makeMotor(int deviceID, String name, double p, double i, double d, double s, double v,
            double a, double maxV, double maxA, double maxJerk) {
        return makeMotor(deviceID, name, p, i, d, s, v, a, 0, maxV, maxA, maxJerk);
    }

    public static PIDMotor makeMotor(int deviceID, String name, double p, double i, double d, double s, double v,
                                     double a, double g, double maxV, double maxA, double maxJerk) {
        return makeMotor(deviceID, name, p, i, d, s, v, a, 0, maxV, maxA, maxJerk, false);
    }

    public static PIDMotor makeMotor(int deviceID, String name, double p, double i, double d, double s, double v,
            double a, double g, double maxV, double maxA, double maxJerk, boolean enableFOC) {
        PIDMotor motor = new PIDMotor(deviceID, name, p, i, d, s, v, a, g, maxV, maxA, maxJerk, enableFOC);
        motor.init();
        System.out.println("Finished initializing " + name);
        return motor;
    }

    /**
     * Throws an exception if motor failed to initialize.
     */
    private void catchUninit() {
        if (!initialized) {
            new Exception("PIDMotor `" + name + "` has not been initialized! Call `init()` before using the motor!").printStackTrace();
        }
    }

    /**
     * The initialization of the motor, only ever called through the makeMotor
     * function.
     */
    private void init() {
        if (!initialized) {
            sleep();
            resetAll();
            sleep();
            // TODO: Once we are sure that all the motors are going in the right direction, set a reasonable current
            // limit for all motors and remove following line.
            // setCurrentLimit(11);
            sleep();
            // putPIDF();
            sleep();
            updatePIDF();
            sleep();
            setIdleCoastMode();
            initialized = true;
        }
    }

    public void sleep() {
        try {
            final int sleepTime = 20;
            Thread.sleep(sleepTime);
        } catch (InterruptedException ignored) {}
    }

    public void setIdleCoastMode() {
        StatusCode code = motor.setNeutralMode(NeutralModeValue.Coast);
        if (!code.isOK()) {
            System.err.printf("Error setting coast mode (%s): %s\n", name, code.getDescription());
        }
    }

    public void setIdleBrakeMode() {
        StatusCode code = motor.setNeutralMode(NeutralModeValue.Brake);
        if (!code.isOK()) {
            System.err.printf("Error setting brake mode (%s): %s\n", name, code.getDescription());
        }
    }

    /**
     * Puts this motor's PIDF values to the SmartDashboard.
     */
    public void putPIDF() {
        SmartDashboard.putNumber(name + " P", p);
        SmartDashboard.putNumber(name + " I", i);
        SmartDashboard.putNumber(name + " D", d);
        SmartDashboard.putNumber(name + " S", s);
        SmartDashboard.putNumber(name + " V", v);
        SmartDashboard.putNumber(name + " A", a);
        SmartDashboard.putNumber(name + " MaxV", maxV);
        SmartDashboard.putNumber(name + " MaxA", maxA);
        SmartDashboard.putNumber(name + " MaxJ", maxJerk);
    }

    /**
     * Puts the position and velocity values to the SmartDashboard.
     */
    public void putPV() {
        SmartDashboard.putNumber(name + " Position", getPosition());
        SmartDashboard.putNumber(name + " Velocity", getVelocity());
    }

    public void putP() {
        SmartDashboard.putNumber(name + " Position", getPosition());
    }

    /**
     * Sets the PIDF values for this motor. Call `updatePIDF` to send the values to
     * the motor
     * controller.
     * 
     * @param p The new proportional coefficient.
     * @param i The new integral coefficient.
     * @param d The new derivative coefficient.
     */
    public void setPIDF(double p, double i, double d, double s, double v, double a, double maxV, double maxA, double maxJerk) {
        catchUninit();
        this.p = p;
        this.i = i;
        this.d = d;
        this.s = s;
        this.v = v;
        this.a = a;
        this.maxV = maxV;
        this.maxA = maxA;
        this.maxJerk = maxJerk;
        updatePIDF();
    }

    /**
     * Fetches the PIDF values from the SmartDashboard. Call `updatePIDF` to send
     * the values to the motor controller.
     */
    public void fetchPIDFFromDashboard() {
        catchUninit();
        setPIDF(SmartDashboard.getNumber(name + " P", p),
                SmartDashboard.getNumber(name + " I", i),
                SmartDashboard.getNumber(name + " D", d),
                SmartDashboard.getNumber(name + " S", s),
                SmartDashboard.getNumber(name + " V", v),
                SmartDashboard.getNumber(name + " A", a),
                SmartDashboard.getNumber(name + " maxA", maxA),
                SmartDashboard.getNumber(name + " maxV", maxV),
                SmartDashboard.getNumber(name + " maxJerk", maxJerk)
                );
    }

    /**
     * Sends the PIDF values to the motor controller. Call when PIDF values are
     * changed.
     */
    public void updatePIDF() {
        // set slot 0 gains
        talonFXConfigs.Slot0.kP = p; // A position error of 2.5 rotations results in 12 V output
        talonFXConfigs.Slot0.kI = i; // no output for integrated error
        talonFXConfigs.Slot0.kD = d; // A velocity error of 1 rps results in 0.1 V

        talonFXConfigs.Slot0.kS = s; // Add 0.25 V output to overcome static friction
        talonFXConfigs.Slot0.kV = v; // A velocity target of 1 rps results in 0.12 V output
        talonFXConfigs.Slot0.kA = a; // An acceleration of 1 rps/s requires 0.01 V output
        talonFXConfigs.Slot0.kG = g;

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = maxV; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = maxA; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = maxJerk; // Target jerk of 1600 rps/s/s (0.1 seconds)

        StatusCode code = motor.getConfigurator().apply(talonFXConfigs);
        if (!code.isOK()) {
            System.err.printf("Error updating PIDF (%s): %s\n", name, code.getDescription());
        }
    }

    /**
     * Resets the encoder, making its current position 0.
     * DOES NOT WORK RN
     */
    public void resetEncoder() {
        StatusCode status = motor.setPosition(0);
        if (!status.isOK()) {
            System.err.printf("Error resetting position (%s): %s\n", name, status.getDescription());
        }
    }

    public void follow(PIDMotor other, boolean inverted) {
        StatusCode code = motor.setControl(new Follower(other.motor.getDeviceID(), inverted));
        if (!code.isOK()) {
            System.err.printf("Error following (%s): %s\n", name, code.getDescription());
        }
    }

    /**
     * Resets the state of the motor. Call this every time this motor is enabled.
     */
    public void resetAll() {
        resetEncoder();
        sleep();
    }

    /**
     * Sets the motor's target to a given unit value.
     */
    public void setTarget(double target) {
        catchUninit();
        this.target = target;

        motionMagicVoltage.withPosition(target);

        StatusCode code = motor.setControl(motionMagicVoltage);
        if (!code.isOK()) {
            System.err.printf("error setting target (%s): %s\n", name, code.getDescription());
        }
    }

    /**
     * Sets the motor's target to a given unit value.
     */
    public void setTarget(double target, boolean enableFOC) {
        motionMagicVoltage.withEnableFOC(enableFOC);
        setTarget(target);
    }

    /**
     * Sets the motor's target with a max acceleration
     */
    public void setTarget(double target, double acceleration) {
        catchUninit();
        this.target = target;

        dynamicMotionMagicVoltage.withPosition(target);
        dynamicMotionMagicVoltage.withVelocity(maxV);
        dynamicMotionMagicVoltage.withAcceleration(acceleration);

        motor.setControl(dynamicMotionMagicVoltage);
        StatusCode code = motor.setControl(dynamicMotionMagicVoltage);
        if (!code.isOK()) {
            System.err.printf("error setting target with acceleration (%s): %s\n", name, code.getDescription());
        }
    }
    public void setTarget(double target,  double velocity, double acceleration) {
        catchUninit();
        this.target = target;

        dynamicMotionMagicVoltage.withPosition(target);
        dynamicMotionMagicVoltage.withVelocity(velocity);
        dynamicMotionMagicVoltage.withAcceleration(acceleration);

        motor.setControl(dynamicMotionMagicVoltage);
        StatusCode code = motor.setControl(dynamicMotionMagicVoltage);
        if (!code.isOK()) {
            System.err.printf("error setting target with acceleration (%s): %s\n", name, code.getDescription());
        }
    }

    /**
     * Sets the motor's target with a max acceleration
     */
    public void setTarget(double target, double acceleration, boolean enableFOC) {
        dynamicMotionMagicVoltage.withEnableFOC(enableFOC);
        setTarget(target, acceleration);
    }

    /**
     * Sets the motor to a given speed as a fraction of the maximum output,
     * overriding the PID controller.
     *
     * @param speed A fraction from -1 to 1 specifying the power to set this motor
     *              to.
     */
    public void setPercentOutput(double speed) {
        catchUninit();
        StatusCode code = motor.setControl(dutyCycleOut.withOutput(speed));
        if (!code.isOK()) {
            System.err.printf("Error setting percent output (%s): %s\n", name, code.getDescription());
        }
    }

    /**
     * Sets the motor to a given speed as a fraction of the maximum output,
     * overriding the PID controller.
     * 
     * @param speed A fraction from -1 to 1 specifying the power to set this motor
     *              to.
     */
    public void setPercentOutput(double speed, boolean enableFOC) {
        dutyCycleOut.withEnableFOC(enableFOC);
        setPercentOutput(speed);
    }

    /**
     * Sets this motor to have inverse rotation.
     */
    public void setInverted(InvertedValue value) {
        // motor.setInverted(state);
        talonFXConfigs.MotorOutput = new MotorOutputConfigs().withInverted(value);
        StatusCode code = motor.getConfigurator().apply(talonFXConfigs);
        if (!code.isOK()) {
            System.err.printf("Error setInverted (%s): %s\n", name, code.getDescription());
        }
    }

    /**
     * Gets the encoder's current position.
     * 
     * @return Position in number of rotations.
     */
    public double getPosition() {
        StatusSignal<Angle> status = motor.getPosition();
        StatusCode code = status.getStatus();
        if (!code.isOK()) {
            System.err.printf("Error getting position (%s): %s\n", name, code.getDescription());
        }
        return status.getValueAsDouble();
    }

    /**
     * Gets whether the current position of the motor is within 10 revolutions of
     * the target position.
     * 
     * @return Whether position at target.
     */
    public boolean atPosition() {
        return atPosition(2);
    }

    public boolean atPosition(double epsilon) {
        return ExtraMath.within(target, getPosition(), epsilon);
    }

    /**
     * Gets the encoder's current velocity.
     * 
     * @return Velocity in rotations per second.
     */
    public double getVelocity() {
        StatusSignal<AngularVelocity> status = motor.getVelocity();
        StatusCode code = status.getStatus();
        if (!code.isOK()) {
            System.err.printf("Error getting velocity (%s): %s\n", name, code.getDescription());
        }
        return status.getValueAsDouble();
    }

    public boolean atVelocity(double threshold) {
        return ExtraMath.within(target, getVelocity(), threshold);
    }

    /**
     * Sets the current limit of the motor.
     * 
     * @param limit Current limit in amps.
     */
    public void setCurrentLimit(int limit) {
        var limitConfigs = new CurrentLimitsConfigs();

        // enable stator current limit
        limitConfigs.StatorCurrentLimit = limit;
        limitConfigs.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits = limitConfigs;

        StatusCode code = motor.getConfigurator().apply(talonFXConfigs);
        if (!code.isOK()) {
            System.err.printf("Error setting current limit (%s): %s\n", name, code.getDescription());
        }
    }

    public double getCurrent() {
        StatusSignal<Current> status = motor.getStatorCurrent();
        StatusCode code = status.getStatus();
        if (!code.isOK()) {
            System.err.printf("Error getting current (%s): %s\n", name, code.getDescription());
        }
        return motor.getStatorCurrent().getValueAsDouble();
    }

    public void putCurrent() {
        SmartDashboard.putNumber(name + " Current", getCurrent());
    }
}
