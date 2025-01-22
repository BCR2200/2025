package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClawSubsystem extends SubsystemBase {
  public PIDMotor clawMotor;
   public enum ClawState {
    Eat, WaitTheyDontLoveYouLikeILoveYou, Vomit, EatAlgae;

    public double speed() {
      switch (this) {
        case Eat:
        case EatAlgae:
          return 1.0;
        case WaitTheyDontLoveYouLikeILoveYou:
          return 0.0;
        case Vomit:
          return -1.0;
        default:
          return 0.0;
      }
    }
  }
  ClawState state = ClawState.WaitTheyDontLoveYouLikeILoveYou;


  public ClawSubsystem() {
    clawMotor = PIDMotor.makeMotor(Constants.clawId, "claw", 0, 0, 0, 0, 0, 0, 0, 0, 0);
    clawMotor.setCurrentLimit(30);
  }

  @Override
  public void periodic() {
    clawMotor.setPercentOutput(state.speed());   
  } 

  public void set(ClawState state) {
    this.state = state;
  }

  public void printDashboard() {
    SmartDashboard.putString("Claw State:", state.toString());
    clawMotor.putPIDF();
    clawMotor.putPV();
  }
}
