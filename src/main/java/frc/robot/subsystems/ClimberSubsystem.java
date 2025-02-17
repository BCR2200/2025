package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;
import frc.robot.timing.TimingUtils;

public class ClimberSubsystem extends SubsystemBase {
  public PIDMotor climbMotor;

  public enum ClimbState {
    Down, Up, Off;
  }

  public ClimbState climbState = ClimbState.Off;

    public double speed() {
        switch (climbState) {
            case Up:
                if(climbMotor.getPosition() > 0){ // negative stuff
                    return 0.0;
                } else {
                    return 1;
                }
            case Down:
                if(climbMotor.getPosition() < Constants.CLIMBER_MAX_HEIGHT){
                    return 0.0;
                } else {
                    return -1;
                }
            default:
                return 0.0;
        }
    }

  public ClimberSubsystem() {
    climbMotor = PIDMotor.makeMotor(Constants.CLIMBER_ID, "climber", 2, 0, 0.1, 0.25, 0.12, 0.01, 0.2, 100, 200, 0);
    climbMotor.setCurrentLimit(15);
    climbMotor.setIdleBrakeMode();
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
    return climbMotor.atPosition(1);
  }

  /**
   * Sets a new desired height for the climbers.
   *
   * @param newHeight
   */
  public void setHeight(double newHeight) {
    climbMotor.setTarget(newHeight);
  }

  @Override
  public void periodic() {
    TimingUtils.logDuration("ClimberSubsystem.periodic", () -> {
      climbMotor.setPercentOutput(speed());
      printDashboard();
    });
  }

  public void printDashboard() {
    TimingUtils.logDuration("CliberSubsystem.printDashboard", () -> {
      // SmartDashboard.putBoolean("Climber At Position", this.atPosition());
      // climbMotor.putPIDF();
      climbMotor.putPV();
      SmartDashboard.putNumber("speed", speed());
      SmartDashboard.putString("climbstate", climbState.toString());
    });
  }
}
