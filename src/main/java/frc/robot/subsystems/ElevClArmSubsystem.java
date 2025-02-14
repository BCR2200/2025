package frc.robot.subsystems;

import java.io.UncheckedIOException;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
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
    public  double elevatorPos;
    public  double armPos;

    public ElevArmPosition(double elevatorPos, double armPos) {
      this.elevatorPos = elevatorPos;
      this.armPos = armPos;
    }

    @Override
    public String toString() {
      return "{E: " + elevatorPos + " A: " + armPos + "}";
    }
  }

  public enum RequestState {
    None, CoralLevel1, CoralLevel2, CoralLevel3, CoralLevel4, UnjamStrat1, UnjamStrat2, Barge, Processor, AlgaeBottom,
    AlgaeTop;
  }

  public enum ControlMode {
    Coral, Algae, Climb;
  }

  public RequestState requestState;
  public ControlMode requestMode;

  public static double SAFE_ARM_ELEVATOR = 20.5;
  
  public final static ElevArmPosition HOPPER_POSITION = new ElevArmPosition(0, 9);
  public final static ElevArmPosition INTAKE_POSITION = new ElevArmPosition(0, 0);
  public final static ElevArmPosition SAFE_CORAL_POSITION = new ElevArmPosition(0, SAFE_ARM_ELEVATOR);
  public final static ElevArmPosition SAFE_ALGAE_POSITION = new ElevArmPosition(6, 45);
  public final static ElevArmPosition CORGAE_POSITION = SAFE_CORAL_POSITION;
  public final static ElevArmPosition SAFE_CLIMB_POSITION = SAFE_CORAL_POSITION;
  public final static ElevArmPosition LVL1_POSITION = new ElevArmPosition(28, 45);
  public final static ElevArmPosition LVL2_POSITION = new ElevArmPosition(17, 25.4);
  public final static ElevArmPosition LVL3_POSITION = new ElevArmPosition(41.5, 24.7);
  public final static ElevArmPosition LVL4_POSITION = new ElevArmPosition(94, 32.5);
  public final static ElevArmPosition PICKBOTTOM_POSITION = new ElevArmPosition(18.6, 38);
  public final static ElevArmPosition PICKTOP_POSITION = new ElevArmPosition(53.5, 41);
  public final static ElevArmPosition LVL1_EMOVE_POSITION = new ElevArmPosition(28, SAFE_ARM_ELEVATOR);
  public final static ElevArmPosition LVL2_EMOVE_POSITION = new ElevArmPosition(17, SAFE_ARM_ELEVATOR);
  public final static ElevArmPosition LVL3_EMOVE_POSITION = new ElevArmPosition(41.5, SAFE_ARM_ELEVATOR);
  public final static ElevArmPosition LVL4_EMOVE_POSITION = new ElevArmPosition(94, SAFE_ARM_ELEVATOR);
  public final static ElevArmPosition PICKBOTTOM_EMOVE_POSITION = new ElevArmPosition(18.6, 45);
  public final static ElevArmPosition PICKTOP_EMOVE_POSITION = new ElevArmPosition(53.5, 45);
  public final static ElevArmPosition BARGE_POSITION = new ElevArmPosition(70, 20.5);
  public final static ElevArmPosition BARGE_EMOVE_POSITION = new ElevArmPosition(70, 45);
  public final static ElevArmPosition PROCESSOR_POSITION = SAFE_ALGAE_POSITION;

  public enum ElevArmState {
    Hopper, Intake, SafeCoral,
    UnjamStrat1, UnjamStrat2,
    LvlOne, LvlTwo, LvlThree, LvlFour,
    LvlOneEMove, LvlTwoEMove, LvlThreeEMove, LvlFourEMove,
    CorgaeTransition,
    SafeAlgae, PickBottom, PickTop,
    PickBottomEMove, PickTopEMove, Barge, BargeEMove, Processor,
    SafeClimb;

    public ElevArmPosition position() {
      return switch (this) {
        case Hopper -> HOPPER_POSITION;
        case Intake -> INTAKE_POSITION;
        case SafeCoral -> SAFE_CORAL_POSITION;
        case CorgaeTransition -> CORGAE_POSITION;
        case SafeAlgae -> SAFE_ALGAE_POSITION;
        case SafeClimb -> SAFE_CLIMB_POSITION;
        case LvlOne -> LVL1_POSITION;
        case LvlTwo -> LVL2_POSITION;
        case LvlThree -> LVL3_POSITION;
        case LvlFour -> LVL4_POSITION;
        case PickBottom -> PICKBOTTOM_POSITION;
        case PickTop -> PICKTOP_POSITION;
        case LvlOneEMove -> LVL1_EMOVE_POSITION;
        case LvlTwoEMove -> LVL2_EMOVE_POSITION;
        case LvlThreeEMove -> LVL3_EMOVE_POSITION;
        case LvlFourEMove -> LVL4_EMOVE_POSITION;
        case PickBottomEMove -> PICKBOTTOM_EMOVE_POSITION;
        case PickTopEMove -> PICKTOP_EMOVE_POSITION;
        case Barge -> BARGE_POSITION;
        case BargeEMove -> BARGE_EMOVE_POSITION;
        case Processor -> PROCESSOR_POSITION;
        default -> SAFE_CORAL_POSITION;
      };
    }
  }

  public PIDMotor leftElevatorMotor;
  public PIDMotor rightElevatorMotor;
  public PIDMotor shoulderMotor;
  public PIDMotor clawMotor;

  public DigitalInput hopperBeamBreak;
  public CANdi clawBeamBreak;

  private ClawState clawstate = ClawState.Stop________HammerTime;
  private ElevArmState state = ElevArmState.Hopper;
  public boolean shootLust = false;

  public boolean positionControl;
  public double clawStartPosition;
  public double clawTargetPosition;

  public enum ClawState {
    Eat, Stop________HammerTime, Vomit, EatAlgae, Poop;

    public double speed() {
      return switch (this) {
        case Eat -> 0.15;
        case EatAlgae -> 1.0;
        case Poop -> 1.0;
        case Stop________HammerTime -> 0.0;
        case Vomit -> -1.0;
        default -> 0.0;
      };
    }
  }

  // see images\ZoneDiagram.png
  // bad zones, should never go
  public Zone armHitElevator = new Zone(new ElevArmPosition(1, 1), new ElevArmPosition(1, 1));
  public Zone armHitBumper = new Zone(new ElevArmPosition(1, 1), new ElevArmPosition(1, 1));

  Timer intakeTimer;

  public static final class Zone {
    final public ElevArmPosition min;
    final public ElevArmPosition max;

    public Zone(ElevArmPosition min, ElevArmPosition max) {
      if (max.elevatorPos < min.elevatorPos) {
        throw new UncheckedIOException("Elevator max is less than min", null);
      }
      if (max.armPos < min.armPos) {
        throw new UncheckedIOException("Arm max is less than min", null);
      }
      this.min = min;
      this.max = max;
    }

    public boolean isItIn(ElevArmPosition position) {
      return (position.elevatorPos < max.elevatorPos && position.elevatorPos > min.elevatorPos)
          && (position.armPos < max.armPos && position.armPos > min.armPos);
    }
  }

  public ElevClArmSubsystem() {
    leftElevatorMotor = PIDMotor.makeMotor(Constants.LEFT_ELEVATOR_ID, "left elevator", 2, 0, 0.1, 0.25, 0.12, 0.01, 0.2, 100, 200, 0);
    rightElevatorMotor = PIDMotor.makeMotor(Constants.RIGHT_ELEVATOR_ID, "right elevator", 2, 0, 0.1, 0.25, 0.12, 0.01, 0.2, 100, 200, 0);
    shoulderMotor = PIDMotor.makeMotor(Constants.SHOULDER_ID, "shoulder", 2, 0, 0.1, 0.25, 0.12, 0.01, 60, 200, 0);
    clawMotor = PIDMotor.makeMotor(Constants.CLAW_ID, "claw", 2, 0, 0.1, 0.25, 0.12, 0.01, 60, 200, 0);
    clawMotor.setInverted(InvertedValue.Clockwise_Positive);

    leftElevatorMotor.follow(rightElevatorMotor, true);

    leftElevatorMotor.setCurrentLimit(40);
    rightElevatorMotor.setCurrentLimit(40);
    shoulderMotor.setCurrentLimit(30);
    clawMotor.setCurrentLimit(30);
    clawMotor.setIdleBrakeMode();

    hopperBeamBreak = new DigitalInput(Constants.HOPPER_ID);

    clawBeamBreak = new CANdi(Constants.CLAW_BREAK_ID, "*");
    CANdiConfiguration configs = new CANdiConfiguration();
    clawBeamBreak.getConfigurator().apply(configs);

    requestState = RequestState.None;
    requestMode = ControlMode.Coral;

    intakeTimer = new Timer();
  }

  @Override
  public void periodic() {
    printDashboard();
    // if arm back, claws forward tracker
    boolean coralInClaw = isCoralInClaw();
    boolean coralInHopper = isCoralInHopper();

    switch (state) { // state transitions
      case Hopper:
        switch (requestMode) {
          case Algae:
            state = ElevArmState.CorgaeTransition;
            break;
          case Climb:
            state = ElevArmState.SafeClimb;
            break;
          default:
            break;
        }
        switch (requestState) {
          case UnjamStrat1:
            state = ElevArmState.UnjamStrat1;
            break;
          case UnjamStrat2:
            state = ElevArmState.UnjamStrat2;
            break;
          default:
            break;
        }
        if (coralInHopper) {
          state = ElevArmState.Intake;
          intakeTimer.restart();
          break;
        }
        if (coralInClaw) {
          state = ElevArmState.SafeCoral;
          break;
        }
        break;
      case Intake:
        if (coralInClaw) {
          state = ElevArmState.SafeCoral;
          clawStartPosition = clawMotor.getPosition();
          positionControl = true;
          break;
        }
        if(intakeTimer.get() > 2.5){
          // something triggered intake accidentally
          state = ElevArmState.Hopper;
        }
        switch (requestState) {
          // case CoralLevel1:
          // state = ElevArmState.LvlOneEMove;
          // break;
          // case CoralLevel2:
          // state = ElevArmState.LvlTwoEMove;
          // break;
          // case CoralLevel3:
          // state = ElevArmState.LvlThreeEMove;
          // break;
          // case CoralLevel4:
          // state = ElevArmState.LvlFourEMove;
          // break;
          default:
            break;
        }
        break;
      case SafeCoral:
        if (!coralInClaw) {
          state = conditionalTransition(state, ElevArmState.Hopper);
          break;
        }
        switch (requestState) {
          case CoralLevel1:
            state = conditionalTransition(state, ElevArmState.LvlOneEMove);
            break;
          case CoralLevel2:
            state = conditionalTransition(state, ElevArmState.LvlTwoEMove);
            break;
          case CoralLevel3:
            state = conditionalTransition(state, ElevArmState.LvlThreeEMove);
            break;
          case CoralLevel4:
            state = conditionalTransition(state, ElevArmState.LvlFourEMove);
            break;
          default:
            break;
        }
        switch (requestMode) {
          case Climb:
            state = ElevArmState.SafeClimb;
            break;
          default:
            break;
        }
        break;
      case CorgaeTransition:
        switch (requestMode) {
          case Algae:
            state = conditionalTransition(state, ElevArmState.SafeAlgae);
            break;
          case Coral:
            state = ElevArmState.SafeCoral;
            break;
          default:
            break;
        }
        break;
      case SafeAlgae:
        switch (requestMode) {
          case Coral:
            state = ElevArmState.SafeCoral;
            break;
          case Climb:
            state = ElevArmState.SafeClimb;
            break;
          default:
            break;
        }
        switch (requestState) {
          case AlgaeTop:
            state = conditionalTransition(state, ElevArmState.PickTopEMove);
            break;
          case AlgaeBottom:
            state = conditionalTransition(state, ElevArmState.PickBottomEMove);
            break;
          case Processor:
            state = conditionalTransition(state, ElevArmState.Processor);
            break;
          case Barge:
            state = conditionalTransition(state, ElevArmState.BargeEMove);
            break;
          default:
            break;
        }
        break;
      case SafeClimb:
        switch (requestMode) {
          case Coral:
            state = ElevArmState.Hopper;
            break;
          case Algae:
            state = ElevArmState.SafeAlgae;
            break;
          default:
            break;
        }
        break;
      case Barge:
        switch (requestState) {
          case Barge:
            break;
          default:
            state = ElevArmState.BargeEMove;
            break;
        }
        break;
      case PickBottom:
        switch (requestState) {
          case AlgaeBottom:
            break;
          default:
            state = ElevArmState.PickBottomEMove;
            break;
        }
        break;
      case PickTop:
        switch (requestState) {
          case AlgaeTop:
            break;
          default:
            state = ElevArmState.PickTopEMove;
            break;
        }
        break;
      case Processor:
        switch (requestState) {
          case Processor:
            break;
          default:
            state = ElevArmState.SafeAlgae;
            break;
        }
        break;
      case LvlOne:
        switch (requestState) {
          case CoralLevel1:
            break;
          default:
            state = ElevArmState.LvlOneEMove;
            break;
        }
        break;
      case LvlTwo:
        switch (requestState) {
          case CoralLevel2:
            break;
          default:
            state = ElevArmState.LvlTwoEMove;
            break;
        }
        break;
      case LvlThree:
        switch (requestState) {
          case CoralLevel3:
            break;
          default:
            state = ElevArmState.LvlThreeEMove;
            break;
        }
        break;
      case LvlFour:
        switch (requestState) {
          case CoralLevel4:
            break;
          default:
            state = ElevArmState.LvlFourEMove;
            break;
        }
        break;
      case LvlOneEMove:
        switch (requestState) {
          case CoralLevel1:
            state = conditionalTransition(state, ElevArmState.LvlOne);
            break;
          case CoralLevel2:
            state = ElevArmState.LvlTwoEMove;
            break;
          case CoralLevel3:
            state = ElevArmState.LvlThreeEMove;
            break;
          case CoralLevel4:
            state = ElevArmState.LvlFourEMove;
            break;
          default:
            state = conditionalTransition(state, ElevArmState.SafeCoral);
            break;
        }
        break;
      case LvlTwoEMove:
        switch (requestState) {
          case CoralLevel1:
            state = ElevArmState.LvlOneEMove;
            break;
          case CoralLevel2:
            state = conditionalTransition(state, ElevArmState.LvlTwo);
            break;
          case CoralLevel3:
            state = ElevArmState.LvlThreeEMove;
            break;
          case CoralLevel4:
            state = ElevArmState.LvlFourEMove;
            break;
          default:
            state = conditionalTransition(state, ElevArmState.SafeCoral);
            break;
        }
        break;
      case LvlThreeEMove:
        switch (requestState) {
          case CoralLevel1:
            state = ElevArmState.LvlOneEMove;
            break;
          case CoralLevel2:
            state = ElevArmState.LvlTwoEMove;
            break;
          case CoralLevel3:
            state = conditionalTransition(state, ElevArmState.LvlThree);
            break;
          case CoralLevel4:
            state = ElevArmState.LvlFourEMove;
            break;
          default:
            state = conditionalTransition(state, ElevArmState.SafeCoral);
            break;
        }
        break;
      case LvlFourEMove:
        switch (requestState) {
          case CoralLevel1:
            state = ElevArmState.LvlOneEMove;
            break;
          case CoralLevel2:
            state = ElevArmState.LvlTwoEMove;
            break;
          case CoralLevel3:
            state = ElevArmState.LvlThreeEMove;
            break;
          case CoralLevel4:
            state = conditionalTransition(state, ElevArmState.LvlFour, 30);
            break;
          default:
            state = conditionalTransition(state, ElevArmState.SafeCoral, 7);
            break;
        }
        break;
      case PickBottomEMove:
        switch (requestState) {
          case AlgaeTop:
            state = ElevArmState.PickTopEMove;
            break;
          case AlgaeBottom:
            state = conditionalTransition(state, ElevArmState.PickBottom);
            break;
          default:
            state = conditionalTransition(state, ElevArmState.SafeAlgae);
            break;
        }
        break;
      case PickTopEMove:
        switch (requestState) {
          case AlgaeTop:
            state = conditionalTransition(state, ElevArmState.PickTop);
            break;
          case AlgaeBottom:
            state = ElevArmState.PickBottomEMove;
            break;
          default:
            state = conditionalTransition(state, ElevArmState.SafeAlgae);
            break;
        }
        break;
      case UnjamStrat1:
        switch (requestState) {
          case UnjamStrat1:
            break;
          default:
            state = ElevArmState.Hopper;
            break;
        }
        break;
      case UnjamStrat2:
        switch (requestState) {
          case UnjamStrat2:
            break;
          default:
            state = ElevArmState.Hopper;
            break;
          // if stuck in unjam position check out
        }
        break;
      case BargeEMove:
        switch (requestState) {
          case Barge:
            state = conditionalTransition(state, ElevArmState.Barge);
            break;
          default:
            state = conditionalTransition(state, ElevArmState.SafeAlgae);
            break;
        }
        break;
      // default:
      // System.err.println("something's busted, no state????");
      // break;

    }

    switch (state) { // in state what are we doing with claw
      case Hopper:
      case SafeCoral:
      default:
        clawstate = ClawState.Stop________HammerTime;
        break;
      case Intake:
        clawstate = ClawState.Eat;
        break;
      case SafeAlgae:
      case PickBottom:
      case PickTop:
      case PickBottomEMove:
      case PickTopEMove:
      case BargeEMove:
        clawstate = ClawState.EatAlgae;
        break;
    }

    if (state != ElevArmState.UnjamStrat1 && state != ElevArmState.UnjamStrat2) {
      go(state.position());
    }
    if (shootLust && getEMode() == ControlMode.Coral && state != ElevArmState.SafeCoral
        && state != ElevArmState.Intake) {
      positionControl = false;
      clawstate = ClawState.Poop;
    }
    if (shootLust && getEMode() == ControlMode.Algae && state != ElevArmState.SafeAlgae) {
      clawstate = ClawState.Vomit;
    }
    if (shootLust && getEMode() == ControlMode.Coral && state == ElevArmState.LvlOne) {
      positionControl = false;
      clawstate = ClawState.Vomit;
    }

    if(!coralInClaw){
      positionControl = false;
    }

    clawTargetPosition = (shoulderMotor.getPosition() * -(24.0/73.4)) + clawStartPosition ;

    if(!positionControl){
      clawMotor.setPercentOutput(clawstate.speed());
    } else {
      clawMotor.setTarget(clawTargetPosition);
    }

    if(getEMode() == ControlMode.Algae && requestMode != ControlMode.Coral){
      clawMotor.setCurrentLimit(30);
    } 
    
  }

  ElevArmPosition currentPosStatic = new ElevArmPosition(0, 0);
  // manage positions asked to, only go if safe
  public void go(ElevArmPosition goal) {
    currentPosStatic.elevatorPos = rightElevatorMotor.getPosition();
    currentPosStatic.armPos = shoulderMotor.getPosition();

    if (armHitElevator.isItIn(currentPosStatic) || armHitBumper.isItIn(currentPosStatic)) {
      // current
      System.out
          .println("Currently killing itself! Current Position: " + currentPosStatic.toString() + " Goal Position: "
              + goal.toString());
    }
    if (armHitElevator.isItIn(goal) || armHitBumper.isItIn(goal)) {
      // DO NOT GO - Default to Safe
      rightElevatorMotor.setTarget(SAFE_CORAL_POSITION.elevatorPos);
      shoulderMotor.setTarget(SAFE_CORAL_POSITION.armPos);
      System.out.println("Tried to kill itself! Current Position: " + currentPosStatic.toString() + " Goal Position: "
          + goal.toString());
    } else {
      // in target or adjacent zone, move to direct
      rightElevatorMotor.setTarget(goal.elevatorPos);
      shoulderMotor.setTarget(goal.armPos);
    }
  }

  public boolean isCoralInHopper() {
    return !hopperBeamBreak.get();
  }

  public ElevArmState conditionalTransition(ElevArmState from, ElevArmState to) {
    if (atPosition()) {
      return to;
    }
    return from;
  }

  public ElevArmState conditionalTransition(ElevArmState from, ElevArmState to, double epsilon) {
    if (atPosition(epsilon)) {
      return to;
    }
    return from;
  }

  public boolean isCoralInClaw() {
    // connected to pin 1
    return clawBeamBreak.getS1Closed().getValue();
  }

  public boolean atPosition() {
    return rightElevatorMotor.atPosition(0.25) && shoulderMotor.atPosition(0.25);
  }
  public boolean atPosition(double epsilon) {
    return rightElevatorMotor.atPosition(epsilon) && shoulderMotor.atPosition(epsilon);
  }

  public void requestState(RequestState state) {
    requestState = state;
  }

  public void requestMode(ControlMode mode) {
    requestMode = mode;
  }

  public ControlMode getEMode() {
    switch (state) {
      default:
      case SafeCoral:
      case Hopper:
      case Intake:
      case LvlOne:
      case LvlTwo:
      case LvlThree:
      case LvlFour:
      case LvlOneEMove:
      case LvlTwoEMove:
      case LvlThreeEMove:
      case LvlFourEMove:
      case UnjamStrat1:
      case UnjamStrat2:
        return ControlMode.Coral;

      case Processor:
      case Barge:
      case SafeAlgae:
      case PickTop:
      case PickBottom:
      case PickTopEMove:
      case PickBottomEMove:
        return ControlMode.Algae;

      case SafeClimb:
        return ControlMode.Climb;
    }
  }

  public void printDashboard() {
    SmartDashboard.putString("ElevArm State:", state.toString());
    SmartDashboard.putString("Requested State:", requestState.toString());

    SmartDashboard.putString("Control Mode:", getEMode().toString());
    SmartDashboard.putString("Requested Mode:", requestMode.toString());
    SmartDashboard.putString("Claw State:", clawstate.toString());
    SmartDashboard.putBoolean("Want to Shoot:", shootLust);
    
    SmartDashboard.putBoolean("Coral in Hopper:", isCoralInHopper());
    SmartDashboard.putBoolean("Coral in Claw:", isCoralInClaw());
    
    // SmartDashboard.putNumber("Claw Start Pos:", clawStartPosition);
    // SmartDashboard.putNumber("Claw Target Pos:", clawTargetPosition);
    // SmartDashboard.putBoolean("Position Control:", positionControl);

    // leftElevatorMotor.putPIDF();
    // rightElevatorMotor.putPIDF();
    // shoulderMotor.putPIDF();
    // clawMotor.putPIDF();

   // leftElevatorMotor.putPV();
    rightElevatorMotor.putPV();
    shoulderMotor.putPV();
   // clawMotor.putPV();
  }
}
