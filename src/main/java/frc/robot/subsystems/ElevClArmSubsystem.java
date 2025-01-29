package frc.robot.subsystems;

import java.io.UncheckedIOException;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;


public class ElevClArmSubsystem extends SubsystemBase {
  // Elevator x 2 kraken
  // 13.1 revolutions of elevator = 7.85 inches of travel
  // Shoulder x 1 kraken
  // 73.5 revolutions = 360 degrees (all the way around, not good)
  // Contains beam break in claw
  // beam break in hopper
  
  // w/o coral default to funnel accept -> hopper break -> run wheels and intake
  // against funnel
  // -> claw break -> go to safe coral :thumbsup:
  
  public class ElevArmPosition {
    public double elevatorPos;
    public double armPos;
    
    public ElevArmPosition(double elevatorPos, double armPos) {
      this.elevatorPos = elevatorPos;
      this.armPos = armPos;
    }
  }

  public enum CurrentState {
    Funnel, Intake, Safe, Custom
  }

  public final ElevArmPosition FUNNEL_POSITION = new ElevArmPosition(0, 0);
  public final ElevArmPosition INTAKE_POSITION = new ElevArmPosition(0, 0);
  public final ElevArmPosition SAFE_POSITION = new ElevArmPosition(0, 0);
  
  public PIDMotor leftElevatorMotor;
  public PIDMotor rightElevatorMotor;
  public PIDMotor shoulderMotor;

  public PIDMotor clawMotor;

  public DigitalInput clawBreak;
  public DigitalInput hopperBreak;

  public ElevArmPosition currentElevArmPos;

  public boolean algaeMode = false;
  
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

  ClawState clawstate = ClawState.WaitTheyDontLoveYouLikeILoveYou;

  public Zone zone1 = new Zone(1, new ElevArmPosition(1, 1), new ElevArmPosition(1, 1), new ElevArmPosition(1, 1));
  public Zone zone2 = new Zone(2, new ElevArmPosition(1, 1), new ElevArmPosition(1, 1), new ElevArmPosition(1, 1));
  public Zone zone3 = new Zone(3, new ElevArmPosition(1, 1), new ElevArmPosition(1, 1), new ElevArmPosition(1, 1));
  public Zone zone4 = new Zone(4, new ElevArmPosition(1, 1), new ElevArmPosition(1, 1), new ElevArmPosition(1, 1));
  public Zone zone5 = new Zone(5, new ElevArmPosition(1, 1), new ElevArmPosition(1, 1), new ElevArmPosition(1, 1));


  public class Zone {
    final public int zoneIndex;
    final public ElevArmPosition min;
    final public ElevArmPosition max;
    final public ElevArmPosition safe;

    // adjacent
    public Zone(int zone, ElevArmPosition min, ElevArmPosition max, ElevArmPosition safe) {
      if (max.elevatorPos < min.elevatorPos) {
        throw new UncheckedIOException("Elevator max is less than min", null);
      }
      if (max.armPos < min.armPos) {
        throw new UncheckedIOException("Arm max is less than min", null);
      }
      if (safe.elevatorPos > max.elevatorPos || safe.elevatorPos < min.elevatorPos) {
        throw new UncheckedIOException("Elevator safe position is greater than max or less than min", null);
      }
      if (safe.armPos > max.armPos || safe.armPos < min.armPos) {
        throw new UncheckedIOException("Arm safe position is greater than max or less than min", null);
      }

      zoneIndex = zone;
      this.min = min;
      this.max = max;
      this.safe = safe;
    }

    public boolean isItIn(ElevArmPosition position) {
      return (position.elevatorPos < max.elevatorPos && position.elevatorPos > min.elevatorPos)
          && (position.armPos < max.armPos && position.armPos > min.armPos);
    }
  }

  public boolean atPosition() {
    return rightElevatorMotor.atPosition(5) && shoulderMotor.atPosition(5);
  }

  public ElevClArmSubsystem() {
    leftElevatorMotor = PIDMotor.makeMotor(Constants.LEFT_ELEVATOR_ID, "left elevator", 0, 0, 0, 0, 0, 0, 0, 0, 0);
    rightElevatorMotor = PIDMotor.makeMotor(Constants.RIGHT_ELEVATOR_ID, "right elevator", 0, 0, 0, 0, 0, 0, 0, 0, 0);
    shoulderMotor = PIDMotor.makeMotor(Constants.SHOULDER_ID, "shoulder", 0, 0, 0, 0, 0, 0, 0, 0, 0);
    leftElevatorMotor.follow(rightElevatorMotor, true);

    leftElevatorMotor.setCurrentLimit(30);
    rightElevatorMotor.setCurrentLimit(30);
    shoulderMotor.setCurrentLimit(30);

    clawMotor = PIDMotor.makeMotor(Constants.CLAW_ID, "claw", 0, 0, 0, 0, 0, 0, 0, 0, 0);
    clawMotor.setCurrentLimit(30);

    clawBreak = new DigitalInput(Constants.CLAW_BREAK_ID);
    hopperBreak = new DigitalInput(Constants.HOPPER_ID);

  }

  @Override
  public void periodic() {
    clawMotor.setPercentOutput(clawstate.speed());   
    //this is isn't good
    currentElevArmPos = new ElevArmPosition(rightElevatorMotor.getPosition(), shoulderMotor.getPosition());
    
    boolean coralAbsent = clawBreak.get();
    boolean hopperEmpty = hopperBreak.get();

    CurrentState state = CurrentState.Safe; // current zero ish
    
    switch (state) { // state transitions
      case Funnel:
        if(!hopperEmpty){
          state = CurrentState.Intake;
        }
        break;
      case Intake:
        if(!coralAbsent){
          state = CurrentState.Safe;
        }
        break;
      case Safe:
        if (coralAbsent && !algaeMode) {
          state = CurrentState.Funnel;
        }
        break;
      default:
        break;
    }

    switch (state) { // in state what are we doing
      case Funnel:
        //TODO
        break;
      case Intake:
        //TODO
        break;
      case Safe:
        clawstate = ClawState.WaitTheyDontLoveYouLikeILoveYou; // stop the claw intake
        break;
      default:
        break;
    }

    // go(currentElevArmPos, CurrentState.position);

    // if(coralAbsent && !algaeMode){
    //   go(currentElevArmPos, FUNNEL_POSITION);
    //   if(atPosition()){
        
    //   }
    // } 
    // if(!coralAbsent && state == CurrentState.Safe){

    //   setClaw(ClawState.WaitTheyDontLoveYouLikeILoveYou);
    //   go(currentElevArmPos, SAFE_POSITION); 
    // }
  }


  // return next elevator/arm
  public void go(ElevArmPosition current, ElevArmPosition goal) {

    // idk how else to find the zone
    Zone[] zones = { zone1, zone2, zone3, zone4, zone5 };
    Zone currentZone = null;
    for (Zone zone : zones) {
      if (zone.isItIn(current)) {
        currentZone = zone;
        break;
      }
    }

    Zone targetZone = null;
    for (Zone zone : zones) {
      if (zone.isItIn(goal)) {
        targetZone = zone;
        break;
      }
    }

    int currentZoneIndex = currentZone.zoneIndex;
    int targetZoneIndex = targetZone.zoneIndex;

    if (currentZone == null || targetZoneIndex < 1 || targetZoneIndex > zones.length) {
      System.out.println("What are you doing? Target isn't in a zone??");
      return;
    }

    // if not in the correct zone/adjacent zone
    if (Math.abs(currentZoneIndex - targetZoneIndex) > 1) {
      int nextZoneIndex = currentZoneIndex < targetZoneIndex ? currentZoneIndex + 1 : currentZoneIndex - 1;
      Zone nextZone = zones[nextZoneIndex - 1];

      // go to triangle spots
      rightElevatorMotor.setTarget(nextZone.safe.elevatorPos);
      shoulderMotor.setTarget(nextZone.safe.armPos);

      currentZoneIndex = nextZoneIndex;

      // wait for movement
    } else {
      // in target zone, move to direct
      rightElevatorMotor.setTarget(goal.elevatorPos);
      shoulderMotor.setTarget(goal.armPos);
    }
  }

  // manage positions asked to, only go if safe

  public void printDashboard() {
    SmartDashboard.putString("Claw State:", clawstate.toString());
    SmartDashboard.putBoolean("Hopper Break:", hopperBreak.get());
    SmartDashboard.putBoolean("Claw Break:", clawBreak.get());
    clawMotor.putPIDF();
    clawMotor.putPV();
  }
}
