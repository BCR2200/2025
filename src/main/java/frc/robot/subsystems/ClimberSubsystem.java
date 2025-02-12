package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;

public class ClimberSubsystem extends SubsystemBase {
    public PIDMotor climbMotor;

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
        climbMotor = PIDMotor.makeMotor(Constants.CLIMBER_ID, "climber", 0, 0, 0, 0,0,0,0,0,0);
        climbMotor.setCurrentLimit(1);
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
     * Climber is at position.
     * 
     * @return True/False.
     */
    public boolean atPosition() {
        return climbMotor.atPosition(5);
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
        printDashboard();
    }

    public void printDashboard() {
        // SmartDashboard.putBoolean("Climber At Position", this.atPosition());
        // climbMotor.putPIDF();
        climbMotor.putPV();
    }
}
