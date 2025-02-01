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

  public static class ElevArmPosition {
    public final double elevatorPos;
    public final double armPos;

    public ElevArmPosition(double elevatorPos, double armPos) {
      this.elevatorPos = elevatorPos;
      this.armPos = armPos;
    }
  }

  public final static ElevArmPosition FUNNEL_POSITION = new ElevArmPosition(0, 0);
  public final static ElevArmPosition INTAKE_POSITION = new ElevArmPosition(0, 0);
  public final static ElevArmPosition SAFE_POSITION = new ElevArmPosition(0, 0);
  public final static ElevArmPosition LVL1_POSITION = new ElevArmPosition(0, 0);
  public final static ElevArmPosition LVL2_POSITION = new ElevArmPosition(0, 0);
  public final static ElevArmPosition LVL3_POSITION = new ElevArmPosition(0, 0);
  public final static ElevArmPosition LVL4_POSITION = new ElevArmPosition(0, 0);
  public final static ElevArmPosition PICKBOTTOM_POSITION = new ElevArmPosition(0, 0);
  public final static ElevArmPosition PICKTOP_POSITION = new ElevArmPosition(0, 0);
  public final static ElevArmPosition BARGE_POSITION = new ElevArmPosition(0, 0);
  public final static ElevArmPosition PROCESSOR_POSITION = new ElevArmPosition(0, 0);

  public enum ElevArmState {
    Funnel, Intake, Safe, LvlOne, LvlTwo, LvlThree, LvlFour, PickBottom, PickTop, Barge, Processor;

    public ElevArmPosition position() {
      return switch (this) {
        case Funnel -> FUNNEL_POSITION;
        case Intake -> INTAKE_POSITION;
        case Safe -> SAFE_POSITION;
        case LvlOne -> LVL1_POSITION;
        case LvlTwo -> LVL2_POSITION;
        case LvlThree -> LVL3_POSITION;
        case LvlFour -> LVL4_POSITION;
        case PickBottom -> PICKBOTTOM_POSITION;
        case PickTop -> PICKTOP_POSITION;
        case Barge -> BARGE_POSITION;
        case Processor -> PROCESSOR_POSITION;
        default -> SAFE_POSITION;
      };
    }
  }

  public PIDMotor leftElevatorMotor;
  public PIDMotor rightElevatorMotor;
  public PIDMotor shoulderMotor;
  public PIDMotor clawMotor;

  public DigitalInput clawBeamBreak;
  public DigitalInput hopperBeamBreak;

  public boolean algaeMode = false;

  private ClawState clawstate = ClawState.Stop________HammerTime; 
  private ElevArmState state = ElevArmState.Safe;
  public boolean shootLust = false;

  public enum ClawState {
    Eat, Stop________HammerTime, Vomit, EatAlgae, Poop;

    public double speed() {
      return switch (this) {
        case Eat -> 1.0;
        case EatAlgae -> 0.5;
        case Poop -> 1.0;
        case Stop________HammerTime -> 0.0;
        case Vomit -> -1.0;
        default -> 0.0;
      };
    }
  }

  // see images\ZoneDiagram.png
  public Zone zone1 = new Zone(1, new ElevArmPosition(1, 1), new ElevArmPosition(1, 1), new ElevArmPosition(1, 1));
  public Zone zone2 = new Zone(2, new ElevArmPosition(1, 1), new ElevArmPosition(1, 1), new ElevArmPosition(1, 1));
  public Zone zone3 = new Zone(3, new ElevArmPosition(1, 1), new ElevArmPosition(1, 1), new ElevArmPosition(1, 1));
  public Zone zone4 = new Zone(4, new ElevArmPosition(1, 1), new ElevArmPosition(1, 1), new ElevArmPosition(1, 1));
  public Zone zone5 = new Zone(5, new ElevArmPosition(1, 1), new ElevArmPosition(1, 1), new ElevArmPosition(1, 1));

  public static final class Zone {
    final public int zoneIndex;
    final public ElevArmPosition min;
    final public ElevArmPosition max;
    final public ElevArmPosition safe;

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

    clawBeamBreak = new DigitalInput(Constants.CLAW_BREAK_ID);
    hopperBeamBreak = new DigitalInput(Constants.HOPPER_ID);
  }

  @Override
  public void periodic() {

    // if arm back, claws forward tracker
    boolean coralInClaw = !isCoralInClaw();
    boolean coralInHopper = !isCoralInHopper();

    switch (state) { // state transitions
      case Funnel:
        if (!coralInHopper) {
          state = ElevArmState.Intake;
        }
        break;
      case Intake:
        if (!coralInClaw) {
          state = ElevArmState.Safe;
        }
        break;
      case Safe:
        if (coralInClaw && !algaeMode) {
          state = ElevArmState.Funnel;
        }
        break;
      default:
        break;
    }

    // ElevArmPosition goal = state.position();

    switch (state) { // in state what are we doing
      case Funnel:
        clawstate = ClawState.Stop________HammerTime;
        break;
      case Intake:
        clawstate = ClawState.Eat;
        break;
      case Safe:
        if (!algaeMode) {
          clawstate = ClawState.Stop________HammerTime;
        } else {
          clawstate = ClawState.EatAlgae;
        }
        break;
      default:
        break;
    }

    go(state.position());
    if(shootLust && (state != ElevArmState.Safe || state != ElevArmState.Intake)){
      clawstate = ClawState.Vomit;
    }
    clawMotor.setPercentOutput(clawstate.speed());
  }

  // manage positions asked to, only go if safe
  public void go(ElevArmPosition goal) {
    ElevArmPosition currentElevArmPos = new ElevArmPosition(rightElevatorMotor.getPosition(),
        shoulderMotor.getPosition());
    Zone[] zones = { zone1, zone2, zone3, zone4, zone5 };
    Zone currentZone = null;
    for (Zone zone : zones) {
      if (zone.isItIn(currentElevArmPos)) {
        currentZone = zone;
        break;
      }
    }
    if (currentZone == null) {
      System.out.println("What are you doing? CurrentElevArmPos isn't in a zone??");
      return;
    }

    Zone targetZone = null;
    for (Zone zone : zones) {
      if (zone.isItIn(goal)) {
        targetZone = zone;
        break;
      }
    }
    if (targetZone == null) {
      System.out.println("What are you doing? Target isn't in a zone??");
      return;
    }

    int currentZoneIndex = currentZone.zoneIndex;
    int targetZoneIndex = targetZone.zoneIndex;

    // if not in the correct zone/adjacent zone
    if (Math.abs(currentZoneIndex - targetZoneIndex) > 1) {
      int nextZoneIndex = currentZoneIndex < targetZoneIndex ? currentZoneIndex + 1 : currentZoneIndex - 1;
      Zone nextZone = zones[nextZoneIndex];

      // go to triangle (safe) spots
      rightElevatorMotor.setTarget(nextZone.safe.elevatorPos);
      shoulderMotor.setTarget(nextZone.safe.armPos);

      // wait for movement (next iteration of periodic()
    } else {
      // in target or adjacent zone, move to direct
      rightElevatorMotor.setTarget(goal.elevatorPos);
      shoulderMotor.setTarget(goal.armPos);
    }
  }

  public boolean isCoralInHopper() {
    return !hopperBeamBreak.get();
  }

  public boolean isCoralInClaw() {
    return !clawBeamBreak.get();
  }

  public boolean atPosition() {
    return rightElevatorMotor.atPosition(5) && shoulderMotor.atPosition(5);
  }

  public void printDashboard() {
    SmartDashboard.putString("ElevArm State:", state.toString());
    SmartDashboard.putBoolean("Algae Mode:", algaeMode);
    SmartDashboard.putString("Claw State:", clawstate.toString());
    SmartDashboard.putBoolean("Coral in Hopper:", isCoralInHopper());
    SmartDashboard.putBoolean("Coral in Claw:", isCoralInClaw());
    clawMotor.putPIDF();
    clawMotor.putPV();
  }
}
