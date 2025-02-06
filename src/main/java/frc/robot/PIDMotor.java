package frc.robot;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
    double p, i, d, s, v, a;
    double maxV;
    double maxA;
    double maxJerk;

    // This can be used as both position and velocity target
    double target = 0.0;

    final TalonFXConfiguration talonFXConfigs;
    final TalonFX motor;

    private PIDMotor(int deviceID, String name, double p, double i, double d, double s, double v, double a, double maxV,
            double maxA, double maxJerk) {
        this.name = name;
        this.p = p;
        this.i = i;
        this.d = d;
        this.s = s;
        this.v = v;
        this.a = a;
        this.maxV = maxV;
        this.maxA = maxA;
        this.maxJerk = maxJerk;

        motor = new TalonFX(deviceID);
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
        PIDMotor motor = new PIDMotor(deviceID, name, p, i, d, s, v, a, maxV, maxA, maxJerk);
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
            setCurrentLimit(10);
            sleep();
            putPIDF();
            sleep();
            updatePIDF();
            sleep();
            motor.setNeutralMode(NeutralModeValue.Brake);
            setIdleBrakeMode();
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
        motor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setIdleBrakeMode() {
        motor.setNeutralMode(NeutralModeValue.Brake);
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

        // set Motion Magic settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = maxV; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = maxA; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = maxJerk; // Target jerk of 1600 rps/s/s (0.1 seconds)

        motor.getConfigurator().apply(talonFXConfigs);
    }

    /**
     * Resets the encoder, making its current position 0.
     * DOES NOT WORK RN
     */
    public void resetEncoder() {
        motor.setPosition(0);
    }

    public void follow(PIDMotor other, boolean inverted) {
        motor.setControl(new Follower(other.motor.getDeviceID(), inverted));
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
        this.target = target;
        
        final MotionMagicExpoVoltage m_request = new MotionMagicExpoVoltage(target);

        // set target position to target rotations
        motor.setControl(m_request);
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
        motor.set(speed);
    }

    /**
     * Sets this motor to have inverse rotation.
     * 
     * @param state Whether or not to make this motor inverted.
     */
    public void setInverted(boolean state) {
        // motor.setInverted(state);
        motor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
    }

    /**
     * Gets the encoder's current position.
     * 
     * @return Position in number of rotations.
     */
    public double getPosition() {
        return motor.getPosition().getValueAsDouble(); 
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
        return motor.getVelocity().getValueAsDouble();
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

        motor.getConfigurator().apply(limitConfigs);
    }

    public double getCurrent() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    public void putCurrent() {
        SmartDashboard.putNumber(name + " Current", getCurrent());
    }
}
