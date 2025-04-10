package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;
import frc.robot.timing.TimingUtils;

public class ClimberSubsystem extends SubsystemBase {
  public PIDMotor climbMotor;
  public double target;
  double manualAdjustBy = 0.6;

  public enum ManualClimbState {
    Down, Up, Off;
  }

  public enum ClimbHeight {
    Stowed, Engaged, Max;

    public double asDouble() {
      switch (this) {
        case Engaged:
          return Constants.CLIMBER_ENGAGED;
        case Max:
          return Constants.CLIMBER_MAX_HEIGHT;
        default:
        case Stowed:
          return Constants.CLIMBER_STOWED;
      }
    }
  }

  public ManualClimbState climbState = ManualClimbState.Off;

  // public double speed() {
  // switch (climbState) {
  // case Up:
  // if(climbMotor.getPosition() > 0){ // negative stuff
  // return 0.0;
  // } else {
  // return 1;
  // }
  // case Down:
  // if(climbMotor.getPosition() < Constants.CLIMBER_MAX_HEIGHT){
  // return 0.0;
  // } else {
  // return -1;
  // }
  // default:
  // return 0.0;
  // }
  // }

  public ClimberSubsystem() {
    climbMotor = PIDMotor.makeMotor(Constants.CLIMBER_ID, "climber", 2, 0, 0.1, 0.25, 0.12, 0.01, 0.2, 100, 200, 0);
    climbMotor.setCurrentLimit(55);
    climbMotor.setIdleBrakeMode();
    target = 0;
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
    target = newHeight;
  }

  /**
   * Sets a new desired height for the climbers.
   *
   * @param newHeight
   */
  public void setHeight(ClimbHeight newHeight) {
    target = newHeight.asDouble();
  }

  @Override
  public void periodic() {
    TimingUtils.logDuration("ClimberSubsystem.periodic", () -> {
      double currentPosition = position();

      switch (climbState) {
        case Up:
          if (currentPosition > Constants.CLIMBER_MAX_HEIGHT) {
            target =target - manualAdjustBy;
          }
          break;
        case Down:
          if (currentPosition < 0) {
            target = target + manualAdjustBy;
          }
          break;
        default:
          // No manual movement
          break;
      }

      climbMotor.setTarget(target);
    });
  }

  public void printDashboard() {
    TimingUtils.logDuration("ClimberSubsystem.printDashboard", () -> {
      climbMotor.putP();
      SmartDashboard.putNumber("target", target);
    });
  }
}