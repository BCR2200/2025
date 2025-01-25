package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;

public class ClimberSubsystem extends SubsystemBase {
    public PIDMotor climbMotor = PIDMotor.makeMotor(Constants.CLIMBER_ID, "Climber", 0, 0, 0, 0,0,0,0,0,0);

    public enum ClimbState {
        Down, Up;

        public double height(){
            switch(this){
                case Down: return Constants.CLIMBER_DOWN_HEIGHT; // constant pain
                case Up: return Constants.CLIMBER_UP_HEIGHT;
                default: return 0;
            }
        }
    }

    public ClimberSubsystem() {
        climbMotor.setCurrentLimit(30);
    }

    /**
     * Position of the climber.
     * 
     * @return The encoder position of the climber motor.
     */
    public double position() {
        return climbMotor.getPosition();
    }

    /**
     * Sets a new desired height for the climbers.
     * 
     * @param newHeight
     */
    public void setHeight(double newHeight) {
        climbMotor.setTarget(newHeight);
    }

    public void setHeight(ClimbState state) {
        setHeight(state.height());
    }

    @Override
    public void periodic() {
        
    }
}
