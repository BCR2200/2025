package frc.robot.subsystems;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;
import frc.robot.timing.TimingUtils;
// slow down arm for algae processor/stow
public class ElevClArmSubsystem extends SubsystemBase {
  private final DataLog log = DataLogManager.getLog();
  private final DoubleLogEntry clawTargetLog = new DoubleLogEntry(log, "/ElevClArmSubsystem/ClawTarget");
  private final DoubleLogEntry clawStartLog = new DoubleLogEntry(log, "/ElevClArmSubsystem/ClawStart");
  private final DoubleLogEntry elevatorPositionLog = new DoubleLogEntry(log, "/ElevClArmSubsystem/Elevatorpos");
  private final DoubleLogEntry armPositionLog = new DoubleLogEntry(log, "/ElevClArmSubsystem/ArmPos");
  private final BooleanLogEntry positionControlLog = new BooleanLogEntry(log, "/ElevClArmSubsystem/PositionControl");
  private final BooleanLogEntry coralInHopperLog = new BooleanLogEntry(log, "/ElevClArmSubsystem/CoralInHopper");
  private final BooleanLogEntry coralInClawLog = new BooleanLogEntry(log, "/ElevClArmSubsystem/CoralInClaw");
  private final StringLogEntry stateLog = new StringLogEntry(log, "/ElevClArmSubsystem/State");
  private final StringLogEntry requestStateLog = new StringLogEntry(log, "/ElevClArmSubsystem/RequestState");
  private final StringLogEntry requestModeLog = new StringLogEntry(log, "/ElevClArmSubsystem/RequestMode");
  private final StringLogEntry clawStateLog = new StringLogEntry(log, "/ElevClArmSubsystem/ClawState");
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
    public double elevatorPos;
    public double armPos;

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
    None, CoralLevel1, CoralLevel2, CoralLevel3, CoralLevel4, UnjamStrat1, UnjamStrat2, Barge, BargePlace, Processor, AlgaeBottom,
    AlgaeTop, UnlockClimb;

    public ElevArmState finaleState() {
      return switch (this) {
        case None -> ElevArmState.SafeCoral;
        case AlgaeBottom -> ElevArmState.PickBottom;
        case AlgaeTop -> ElevArmState.PickTop;
        case Barge -> ElevArmState.BargeWindup;
        case BargePlace -> ElevArmState.BargePlace;
        case CoralLevel1 -> ElevArmState.LvlOne;
        case CoralLevel2 -> ElevArmState.LvlTwo;
        case CoralLevel3 -> ElevArmState.LvlThree;
        case CoralLevel4 -> ElevArmState.LvlFour;
        case Processor -> ElevArmState.Processor;
        case UnjamStrat1 -> ElevArmState.UnjamStrat1;
        case UnjamStrat2 -> ElevArmState.UnjamStrat2;
        case UnlockClimb -> ElevArmState.SafeClimb;
      };
    }
  }

  public enum ControlMode {
    Coral, Algae, Climb
  }

  public RequestState requestState = RequestState.None;
  public ControlMode requestMode = ControlMode.Coral;

  public static double SAFE_CORAL_ARM = 20.5;
  public static double SAFE_ALGAE_ARM = 31.6;

  // public final static ElevArmPosition HOPPER_POSITION = new ElevArmPosition(0, 9);
  // public final static ElevArmPosition INTAKE_POSITION = new ElevArmPosition(0, 1.0);
  // public final static ElevArmPosition SAFE_CORAL_POSITION = new ElevArmPosition(0, SAFE_CORAL_ARM);
  // public final static ElevArmPosition SAFE_ALGAE_POSITION = new ElevArmPosition(0, 23);
  // public final static ElevArmPosition SAFE_ALGAE_EMOVE_POSITION = new ElevArmPosition(0, 28);
  // public final static ElevArmPosition CORGAE_POSITION = SAFE_CORAL_POSITION;
  // public final static ElevArmPosition SAFE_CLIMB_POSITION = SAFE_CORAL_POSITION;
  // public final static ElevArmPosition PROCESSOR_POSITION = new ElevArmPosition(6.5, 45.7);

  // public final static ElevArmPosition LVL1_POSITION = new ElevArmPosition(15, 45);
  // public final static ElevArmPosition LVL1_EMOVE_POSITION = new ElevArmPosition(0, SAFE_CORAL_ARM);
  // public final static ElevArmPosition LVL2_POSITION = new ElevArmPosition(17, 25.4);
  // public final static ElevArmPosition LVL2_EMOVE_POSITION = new ElevArmPosition(17, SAFE_CORAL_ARM);
  // public final static ElevArmPosition LVL3_POSITION = new ElevArmPosition(41.5, 24.7);
  // public final static ElevArmPosition LVL3_EMOVE_POSITION = new ElevArmPosition(41.5, SAFE_CORAL_ARM);
  // public final static ElevArmPosition LVL4_POSITION = new ElevArmPosition(97, 32);
  // public final static ElevArmPosition LVL4_EMOVE_POSITION = new ElevArmPosition(97, SAFE_CORAL_ARM);
  
  // public final static ElevArmPosition PICKBOTTOM_POSITION = new ElevArmPosition(21, 38);
  // public final static ElevArmPosition PICKBOTTOM_EMOVE_POSITION = new ElevArmPosition(8, 38);
  // public final static ElevArmPosition PICKTOP_POSITION = new ElevArmPosition(45, 28);
  // public final static ElevArmPosition PICKTOP_EMOVE_POSITION = new ElevArmPosition(53.5, 38);
  // public final static ElevArmPosition BARGE_POSITION = new ElevArmPosition(103, 17.6);
  // public final static ElevArmPosition BARGE_EMOVE_POSITION = new ElevArmPosition(103, SAFE_ALGAE_ARM);
  // public final static ElevArmPosition HOPPER_POSITION = new ElevArmPosition(0, 9);

  static double elevatorRatio = 0.69625; // implied

  public final static ElevArmPosition HOPPER_POSITION = new ElevArmPosition(0, 9);
  public final static ElevArmPosition INTAKE_POSITION = new ElevArmPosition(0, 1.0);
  public final static ElevArmPosition SAFE_CORAL_POSITION = new ElevArmPosition(0, SAFE_CORAL_ARM);
  public final static ElevArmPosition SAFE_ALGAE_POSITION = new ElevArmPosition(0, 23);
  public final static ElevArmPosition SAFE_ALGAE_EMOVE_POSITION = new ElevArmPosition(0, 28);
  public final static ElevArmPosition CORGAE_POSITION = SAFE_CORAL_POSITION;
  public final static ElevArmPosition SAFE_CLIMB_POSITION = SAFE_CORAL_POSITION;
  public final static ElevArmPosition PROCESSOR_POSITION = new ElevArmPosition(6.5 * elevatorRatio, 47);
  
  public final static ElevArmPosition LVL1_POSITION = new ElevArmPosition(15 * elevatorRatio, 45);
  public final static ElevArmPosition LVL1_EMOVE_POSITION = new ElevArmPosition(0, SAFE_CORAL_ARM);
  public final static ElevArmPosition LVL2_POSITION = new ElevArmPosition(17 * elevatorRatio, 25.4);
  public final static ElevArmPosition LVL2_EMOVE_POSITION = new ElevArmPosition(17 * elevatorRatio, SAFE_CORAL_ARM);
  public final static ElevArmPosition LVL3_POSITION = new ElevArmPosition(41.5 * elevatorRatio, 24.7);
  public final static ElevArmPosition LVL3_EMOVE_POSITION = new ElevArmPosition(41.5 * elevatorRatio, SAFE_CORAL_ARM);
  public final static ElevArmPosition LVL4_POSITION = new ElevArmPosition(97 * elevatorRatio, 32);
  public final static ElevArmPosition LVL4_EMOVE_POSITION = new ElevArmPosition(97 * elevatorRatio, SAFE_CORAL_ARM);

  public final static ElevArmPosition PICKBOTTOM_POSITION = new ElevArmPosition(21 * elevatorRatio, 38);
  public final static ElevArmPosition PICKBOTTOM_EMOVE_POSITION = new ElevArmPosition(8 * elevatorRatio, 38);
  public final static ElevArmPosition PICKTOP_POSITION = new ElevArmPosition(45 * elevatorRatio, 28);
  public final static ElevArmPosition PICKTOP_EMOVE_POSITION = new ElevArmPosition(53.5 * elevatorRatio, 38);
  public final static ElevArmPosition BARGE_PLACE_POSITION = new ElevArmPosition(105 * elevatorRatio, 16.8);
  public final static ElevArmPosition BARGE_EPLACE_POSITION = new ElevArmPosition(105 * elevatorRatio, 28);
  public final static ElevArmPosition BARGE_POSITION = new ElevArmPosition(103 * elevatorRatio, 20);
  public final static ElevArmPosition BARGE_EMOVE_POSITION = new ElevArmPosition(103 * elevatorRatio, 38);
  public final static ElevArmPosition BARGE_WINDUP_POSITION = new ElevArmPosition(103 * elevatorRatio, 38);
  private final static double bargeShootingTimeWaitForThisLong = 0.08;

  public enum ElevArmState {
    Hopper, Intake, SafeCoral,
    UnjamStrat1, UnjamStrat2,
    LvlOne, LvlTwo, LvlThree, LvlFour,
    LvlOneEMove, LvlTwoEMove, LvlThreeEMove, LvlFourEMove,
    CorgaeTransition,
    SafeAlgae, PickBottom, PickTop, SafeAlgaeEMove,
    PickBottomEMove, PickTopEMove, Barge, BargeWindup, BargeEMove, Processor,
    BargeEPlace, BargePlace,
    SafeClimb, UnlockClimb;

    public ElevArmPosition position() {
      return switch (this) {
        case Hopper -> HOPPER_POSITION;
        case Intake, UnjamStrat1 -> INTAKE_POSITION;
        case SafeCoral -> SAFE_CORAL_POSITION;
        case CorgaeTransition -> CORGAE_POSITION;
        case SafeAlgae -> SAFE_ALGAE_POSITION;
        case SafeAlgaeEMove -> SAFE_ALGAE_EMOVE_POSITION;
        case SafeClimb -> SAFE_CLIMB_POSITION;
        case UnlockClimb -> SAFE_CLIMB_POSITION;
        case LvlOne -> LVL1_POSITION;
        case LvlTwo -> LVL2_POSITION;
        case LvlThree -> LVL3_POSITION;
        case LvlFour -> LVL4_POSITION;
        case PickBottom -> PICKBOTTOM_POSITION;
        case PickTop -> PICKTOP_POSITION;
        case LvlOneEMove -> LVL1_EMOVE_POSITION;
        case LvlTwoEMove, UnjamStrat2 -> LVL2_EMOVE_POSITION;
        case LvlThreeEMove -> LVL3_EMOVE_POSITION;
        case LvlFourEMove -> LVL4_EMOVE_POSITION;
        case PickBottomEMove -> PICKBOTTOM_EMOVE_POSITION;
        case PickTopEMove -> PICKTOP_EMOVE_POSITION;
        case Barge -> BARGE_POSITION;
        case BargePlace -> BARGE_PLACE_POSITION;
        case BargeEPlace -> BARGE_EPLACE_POSITION;
        case BargeWindup -> BARGE_WINDUP_POSITION;
        case BargeEMove -> BARGE_EMOVE_POSITION;
        case Processor -> PROCESSOR_POSITION;
      };
    }

    public ControlMode getControlMode() {
      return switch (this) {
        case SafeCoral, Hopper, Intake, LvlOne, LvlTwo, LvlThree, LvlFour, LvlOneEMove, LvlTwoEMove, LvlThreeEMove,
            LvlFourEMove, UnjamStrat1, UnjamStrat2 ->
          ControlMode.Coral;
        case Processor, Barge, BargeWindup, SafeAlgae, PickTop, PickBottom, PickTopEMove, PickBottomEMove, BargeEMove,
            CorgaeTransition, SafeAlgaeEMove, BargeEPlace, BargePlace ->
          ControlMode.Algae;
        case SafeClimb, UnlockClimb -> ControlMode.Climb;
      };
    }
  }

  public PIDMotor leftElevatorMotor;
  public PIDMotor rightElevatorMotor;
  public PIDMotor shoulderMotor;
  public PIDMotor clawMotor;

  public DigitalInput hopperBeamBreak;
  public CANdi clawBeamBreaks;

  private ClawState clawstate = ClawState.Stop________HammerTime;
  public ElevArmState state = ElevArmState.Hopper;
  public boolean shootLust = false;
  public boolean suck = false;

  public boolean positionControl = false;
  public double clawStartPosition;
  public double armStartPosition;
  public double clawTargetPosition;

  public enum ClawState {
    Eat, Stop________HammerTime, Vomit, EatAlgae, Poop, Drool, LazyBowelSyndrome;

    public double speed() {
      return switch (this) {
        case Eat -> 0.6;
        case EatAlgae -> 0.7;
        case Poop -> 1.0;
        case Stop________HammerTime -> 0.0;
        case Vomit -> -1.0;
        case Drool -> -0.8;
        case LazyBowelSyndrome -> 0.5;
      };
    }
  }

  Timer intakeTimer;
  Timer bargeTimer;
  public boolean manualCoral = false;

  final int normalShoulderCurrentLimit = 30;
  final int softShoulderCurrentLimit = 10;

  final int normalClawCurrentLimit = 30;
  final int algaeClawCurrentLimit = 35;

  public ElevClArmSubsystem() {
    leftElevatorMotor = PIDMotor.makeMotor(Constants.LEFT_ELEVATOR_ID, "left elevator", 2.5, 0, 0.1, 0.25, 0.1, 
        0.01, 0.2, 100, 300, 0);
    rightElevatorMotor = PIDMotor.makeMotor(Constants.RIGHT_ELEVATOR_ID, "right elevator", 2.5, 0, 0.1, 0.25, 0.1,
        0.01, 0.2, 100, 300, 0);
    shoulderMotor = PIDMotor.makeMotor(Constants.SHOULDER_ID, "shoulder", 2, 0, 0.1, 0.25, 0.12, 0.01, 100, 350, 0);
    clawMotor = PIDMotor.makeMotor(Constants.CLAW_ID, "claw", 2, 0, 0.1, 0.25, 0.12, 0.01, 100, 500, 0);
    clawMotor.setInverted(InvertedValue.Clockwise_Positive);
    
    leftElevatorMotor.follow(rightElevatorMotor, true);

    leftElevatorMotor.setCurrentLimit(60);
    rightElevatorMotor.setCurrentLimit(60);
    shoulderMotor.setCurrentLimit(normalShoulderCurrentLimit);
    clawMotor.setCurrentLimit(normalClawCurrentLimit);
    clawMotor.setIdleBrakeMode();

    hopperBeamBreak = new DigitalInput(Constants.HOPPER_ID);

    clawBeamBreaks = new CANdi(Constants.CLAW_BREAK_ID, "*");
    CANdiConfiguration configs = new CANdiConfiguration();
    clawBeamBreaks.getConfigurator().apply(configs);

    intakeTimer = new Timer();
    bargeTimer = new Timer();
  }

  @Override
  public void periodic() {
    TimingUtils.logDuration("ElevClArmSubsystem.periodic", () -> {

      AtomicBoolean coralInHopper = new AtomicBoolean(false);
      AtomicBoolean coralIsSkibidi = new AtomicBoolean(false);
      AtomicBoolean coralEnteredClaw = new AtomicBoolean(false);
      TimingUtils.logDuration("ElevClArmSubsystem.update_coral_dio", () -> {
        // if arm back, claws forward tracker
        coralIsSkibidi.set(isCoralSkibidi());
        coralEnteredClaw.set(isCoralEnteredClaw());
        coralInHopper.set(isCoralInHopper());
      });

      TimingUtils.logDuration("ElevClArmSubsystem.first_switch", () -> {
        switch (state) { // state transitions
          case Hopper:
            switch (requestMode) {
              case Algae:
                state = ElevArmState.CorgaeTransition;
                clawMotor.setCurrentLimit(algaeClawCurrentLimit);
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
                shoulderMotor.setCurrentLimit(softShoulderCurrentLimit);
                break;
              case UnjamStrat2:
                manualCoral = true;
                state = ElevArmState.SafeCoral;
                break;
              default:
                break;
            }
            if (coralInHopper.get()) {
              state = ElevArmState.Intake;
              intakeTimer.restart();
              break;
            }
            if (coralEnteredClaw.get() || manualCoral) {
              state = ElevArmState.SafeCoral;
              break;
            }
            break;
          case Intake:
            if (coralEnteredClaw.get() || manualCoral) {
              state = ElevArmState.SafeCoral;
              // clawStartPosition = clawMotor.getPosition();
              // positionControl = true; no position control here anymore, use arm movement to seed coral (make skibidi)
              break;
            }
            if (intakeTimer.get() > 2.5) {
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
            if ( (!coralEnteredClaw.get() && !coralIsSkibidi.get()) && !manualCoral) {
              state = conditionalTransition(state, ElevArmState.Hopper);
              break;
            }
            
            switch (requestState) {
              case UnjamStrat1:
                if (manualCoral) {
                  state = ElevArmState.UnjamStrat1;
                  shoulderMotor.setCurrentLimit(softShoulderCurrentLimit);
                }
                break;
              case UnjamStrat2:
                // if (manualCoral) {
                  state = conditionalTransition(state, ElevArmState.UnjamStrat2);
                // }
                break;
              case CoralLevel1:
              case CoralLevel2:
              case CoralLevel3:
              case CoralLevel4:
                state = conditionalTransition(state, ElevArmState.LvlOneEMove);
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
              case Climb:
                state = ElevArmState.SafeClimb;
                break;
              default:
                break;
            }
            break;
          case SafeAlgae:
            switch (requestMode) {
              case Coral:
                state = ElevArmState.SafeCoral;
                clawMotor.setCurrentLimit(normalClawCurrentLimit);
                break;
              case Climb:
                state = ElevArmState.SafeClimb;
                clawMotor.setCurrentLimit(normalClawCurrentLimit);
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
              case BargePlace:
                state = conditionalTransition(state, ElevArmState.BargeEPlace);
                break;
              default:
                break;
            }
            break;
          case SafeClimb:
            switch (requestState) {
              case UnlockClimb:
                state = ElevArmState.UnlockClimb;
                break;
              default:
                break;
            }
            break;
          case UnlockClimb:
            switch (requestMode) {
              case Coral:
                state = ElevArmState.SafeCoral;
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
                if(!shootLust){
                  bargeTimer.stop();
                  bargeTimer.reset();
                  state = ElevArmState.BargeWindup;
                }
                break;
              default:
                state = ElevArmState.BargeEMove;
                break;
            }
            break;
          case BargePlace:
            switch (requestState) {
              case BargePlace:
                break;
              default:
                state = ElevArmState.BargeEPlace;
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
                state = ElevArmState.SafeAlgaeEMove;
                break;
            }
            break;
          case Processor:
            switch (requestState) {
              case Processor:
                break;
              default:
                state = ElevArmState.SafeAlgaeEMove;
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
            switch (requestMode) {
              case Algae:
                if (!coralEnteredClaw.get() && !coralIsSkibidi.get()) {
                  state = ElevArmState.SafeAlgaeEMove;
                }
                break;
              default:
                break;
            }
            switch (requestState) {
              case CoralLevel1:
                state = ElevArmState.LvlOne; // ignore elev bc it's fine and faster
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
                state = conditionalTransition(state, ElevArmState.SafeCoral, 1);
                break;
            }
            break;
          case LvlTwoEMove:
            switch (requestMode) {
              case Algae:
                if (!coralEnteredClaw.get() && !coralIsSkibidi.get()) {
                  state = ElevArmState.SafeAlgaeEMove;
                }
                break;
              default:
                break;
            }
            switch (requestState) {
              case CoralLevel2:
                state = conditionalTransition(state, ElevArmState.LvlTwo, 10);
                break;
              case CoralLevel3:
                state = ElevArmState.LvlThreeEMove;
                break;
              case CoralLevel4:
                state = ElevArmState.LvlFourEMove;
                break;
              default:
                state = ElevArmState.LvlOneEMove;
                break;
            }
            break;
          case LvlThreeEMove:
            switch (requestMode) {
              case Algae:
                if (!coralEnteredClaw.get() && !coralIsSkibidi.get()) {
                  state = ElevArmState.SafeAlgaeEMove;
                }
                break;
              default:
                break;
            }
            switch (requestState) {
              case CoralLevel2:
                state = ElevArmState.LvlTwoEMove;
                break;
              case CoralLevel3:
                state = conditionalTransition(state, ElevArmState.LvlThree, 10);
                break;
              case CoralLevel4:
                state = ElevArmState.LvlFourEMove;
                break;
              default:
                state = ElevArmState.LvlOneEMove;
                break;
            }
            break;
          case LvlFourEMove:
            switch (requestMode) {
              case Algae:
                if (!coralEnteredClaw.get() && !coralIsSkibidi.get()) {
                  state = ElevArmState.SafeAlgaeEMove;
                }
                break;
              default:
                break;
            }
            switch (requestState) {
              case CoralLevel2:
                state = ElevArmState.LvlTwoEMove;
                break;
              case CoralLevel3:
                state = ElevArmState.LvlThreeEMove;
                break;
              case CoralLevel4:
                state = conditionalTransition(state, ElevArmState.LvlFour, 30*elevatorRatio);
                break;
              default:
                state = conditionalTransition(state, ElevArmState.LvlOneEMove, 10);
                break;
            }
            break;
          case PickBottomEMove:
            switch (requestState) {
              case AlgaeTop:
                state = ElevArmState.PickTopEMove;
                break;
              case Barge:
                state = ElevArmState.BargeEMove;
                break;
              case BargePlace:
                state = ElevArmState.BargeEPlace;
                break;
              case AlgaeBottom:
                state = conditionalTransition(state, ElevArmState.PickBottom, 30*elevatorRatio);
                break;
              default:
                state = conditionalTransition(state, ElevArmState.SafeAlgaeEMove, 30*elevatorRatio);
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
              case Barge:
                state = ElevArmState.BargeEMove;
                break;
              case BargePlace:
                state = ElevArmState.BargeEPlace;
                break;
              default:
                state = conditionalTransition(state, ElevArmState.SafeAlgaeEMove);
                break;
            }
            break;
          case UnjamStrat1:
            switch (requestState) {
              case UnjamStrat1:
                break;
              default:
                state = ElevArmState.Hopper;
                shoulderMotor.setCurrentLimit(normalShoulderCurrentLimit);
                turnOnPosCtrl();
                break;
            }
            break;
          case UnjamStrat2:
            switch (requestState) {
              case UnjamStrat2:
                manualCoral = true;
                break;
              default:
                state = ElevArmState.LvlTwoEMove;
                manualCoral = false;
                turnOnPosCtrl();
                break;
              // if stuck in unjam position check out
            }
            break;
          case BargeEMove:
            switch (requestState) {
              case Barge:
                bargeTimer.stop();
                bargeTimer.reset();
                state = ElevArmState.BargeWindup;
                break;
              case AlgaeBottom:
                state = ElevArmState.PickBottomEMove;
                break;
              case AlgaeTop:
                state = ElevArmState.PickTopEMove;
                break;
              case BargePlace:
                state = ElevArmState.BargeEPlace;
                break;
              default:
                state = conditionalTransition(state, ElevArmState.SafeAlgaeEMove, 10*elevatorRatio);
                break;
            }
            break;
          case BargeEPlace:
            switch (requestState) {
              case BargePlace:
                state = conditionalTransition(state, ElevArmState.BargePlace, 30*elevatorRatio);
                break;
              case Barge:
                state = ElevArmState.BargeEMove;
                break;
              case AlgaeBottom:
                state = ElevArmState.PickBottomEMove;
                break;
              case AlgaeTop:
                state = ElevArmState.PickTopEMove;
                break;
              default:
                state = conditionalTransition(state, ElevArmState.SafeAlgaeEMove, 10*elevatorRatio);
                break;
            }
            break;
          case BargeWindup:
            switch (requestState) {
              case Barge:
                if(shootLust){
                  state = ElevArmState.Barge;
                  bargeTimer.restart();
                }
                break;
              default:
                state = ElevArmState.BargeEMove;
                break;
            }
            break;
          case SafeAlgaeEMove:
            switch (requestState) {
              case AlgaeTop:
                state = conditionalTransition(state, ElevArmState.PickTopEMove);
                break;
              case AlgaeBottom:
                state = ElevArmState.PickBottomEMove;
                break;
              case Barge:
                state = ElevArmState.BargeEMove;
                break;
              case BargePlace:
                state = ElevArmState.BargeEPlace;
                break;
              case Processor:
                state = conditionalTransition(state, ElevArmState.Processor);
                break;
              default:
                state = conditionalTransition(state, ElevArmState.SafeAlgae, 7);
                break;
            }
            break;

        }
      });

      TimingUtils.logDuration("ElevClArmSubsystem.second_switch", () -> {
        switch (state) { // in state what are we doing with claw
          case Hopper:
          case SafeCoral:
          case CorgaeTransition:
          case LvlFour:
          case LvlFourEMove:
          case LvlOne:
          case LvlOneEMove:
          case LvlThree:
          case LvlThreeEMove:
          case LvlTwo:
          case LvlTwoEMove:
          case SafeClimb:
          case UnlockClimb:
          case UnjamStrat2:
            clawstate = ClawState.Stop________HammerTime;
            break;
          case Intake:
          case UnjamStrat1:
          clawstate = ClawState.Eat;
            break;
          case SafeAlgae:
          case PickBottom:
          case PickTop:
          case PickBottomEMove:
          case PickTopEMove:
          case BargeEMove:
          case Processor:
          case Barge:
          case BargePlace:
          case BargeEPlace:
          case BargeWindup:
          case SafeAlgaeEMove:
            clawstate = ClawState.EatAlgae;
            break;
          default:
            throw new IllegalArgumentException("Unexpected value: " + this);
        }
      });
      
      go(state.position());
      ControlMode eMode = getEMode();

      if(coralIsSkibidi.get() && eMode == ControlMode.Coral && !positionControl && clawstate != ClawState.Poop){
        turnOnPosCtrl();
      }

      if (shootLust && eMode == ControlMode.Coral && state != ElevArmState.SafeCoral
          && state != ElevArmState.Intake && state != ElevArmState.Hopper
          || shootLust && eMode == ControlMode.Coral && manualCoral) {
        positionControl = false;
        clawstate = ClawState.Poop;
      } else if (shootLust && eMode == ControlMode.Coral && state == ElevArmState.Hopper) {
        positionControl = false;
        clawstate = ClawState.LazyBowelSyndrome;
      }
      if (suck && eMode == ControlMode.Coral && state != ElevArmState.SafeCoral
          && state != ElevArmState.Intake || suck && eMode == ControlMode.Coral && manualCoral) {
        positionControl = false;
        clawstate = ClawState.Drool;
      }

      if (shootLust && eMode == ControlMode.Algae && (state == ElevArmState.Barge || state == ElevArmState.BargeWindup || state == ElevArmState.BargeEMove)) {
        //timed vomit
        if(bargeTimer.get() > bargeShootingTimeWaitForThisLong){
          clawstate = ClawState.Vomit;
        }
      } else if (shootLust && eMode == ControlMode.Algae && state != ElevArmState.SafeAlgae) {
        clawstate = ClawState.Vomit;
      }

      if (shootLust && eMode == ControlMode.Coral && state == ElevArmState.LvlOne) {
        positionControl = false;
        clawstate = ClawState.Drool;
      }


      if (!coralEnteredClaw.get() && !coralIsSkibidi.get()) {
        positionControl = false;
      }

      clawTargetPosition = ( (shoulderMotor.getPosition() - armStartPosition) * -(24.0 / 73.4)) + clawStartPosition;

      // if(requestState == ElevArmState.LvlOne){
      //   clawTargetPosition -= 5;
      // }
      
      if (!positionControl) {
        clawMotor.setPercentOutput(clawstate.speed());
      } else {
        clawMotor.setTarget(clawTargetPosition);
      }
      stateLog.append(state.toString());
      requestStateLog.append(requestState.toString());
      requestModeLog.append(requestMode.toString());
      clawStateLog.append(clawstate.toString());
      positionControlLog.append(positionControl);
      clawTargetLog.append(clawTargetPosition);
      clawStartLog.append(clawStartPosition);
      elevatorPositionLog.append(rightElevatorMotor.getPosition());
      armPositionLog.append(shoulderMotor.getPosition());
      coralInHopperLog.append(coralInHopper.get());
      coralInClawLog.append(coralIsSkibidi.get());
      // coralInClawLog.append(coralEnteredClaw.get());
    });
  }

  private void turnOnPosCtrl() {
    if (DriverStation.isEnabled()) {
      clawStartPosition = clawMotor.getPosition();
      armStartPosition = shoulderMotor.getPosition();
      positionControl = true;
    }
  }

  double lvl1SlowAccel = 250;
  double algaeArmAccel = 250;
  double algaeYeetVel = 70;
  double algaeYeetAccel = 400;
  // manage positions asked to, only go if safe
  public void go(ElevArmPosition goal) {
    TimingUtils.logDuration("ElevClArmSubsystem.go", () -> {
      if(state == ElevArmState.LvlOne){
        shoulderMotor.setTarget(goal.armPos, lvl1SlowAccel);
      } else if(state == ElevArmState.SafeCoral){
        shoulderMotor.setTarget(goal.armPos, 250);
      } else if(state == ElevArmState.Barge){
        shoulderMotor.setTarget(goal.armPos, algaeYeetVel, algaeYeetAccel);
      } else if(state == ElevArmState.SafeAlgae){
        shoulderMotor.setTarget(goal.armPos, 50, algaeArmAccel);
      } else if (getEMode() == ControlMode.Algae){
        shoulderMotor.setTarget(goal.armPos, algaeArmAccel);
      } else{
        shoulderMotor.setTarget(goal.armPos);
      }
      rightElevatorMotor.setTarget(goal.elevatorPos);
    });
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

  public boolean isCoralEnteredClaw() {
    // connected to pin 2
    return clawBeamBreaks.getS2Closed().getValue(); // s2 is the bottom sensor (new added to candi)
  }

  public boolean isCoralSkibidi() {
    // connected to pin 1
    return clawBeamBreaks.getS1Closed().getValue();
  }

  public boolean atPosition() {
    return atPosition(1);
  }

  public boolean atPosition(double epsilon) {
    AtomicBoolean retVal = new AtomicBoolean(false);
    TimingUtils.logDuration("ElevClArmSubsystem.atPosition", () -> {
      retVal.set(

          rightElevatorMotor.atPosition(epsilon) &&
              shoulderMotor.atPosition(epsilon));
    });
    return retVal.get();
  }

  public boolean atFinalPosition(double epsilon) {
    AtomicBoolean retVal = new AtomicBoolean(false);
    TimingUtils.logDuration("ElevClArmSubsystem.atFinalPosition", () -> {
      retVal.set(
          requestState.finaleState() == state &&
              rightElevatorMotor.atPosition(epsilon) &&
              shoulderMotor.atPosition(epsilon));
    });
    return retVal.get();
  }

  public void requestState(RequestState state) {
    requestState = state;
  }

  public void requestMode(ControlMode mode) {
    requestMode = mode;
  }

  public ControlMode getEMode() {
    AtomicReference<ControlMode> retVal = new AtomicReference<>(ControlMode.Coral);
    TimingUtils.logDuration("ElevClArmSubsystem.getEMode", () -> {
      retVal.set(state.getControlMode());
    });
    return retVal.get();
  }

  public void printDashboard() {
    TimingUtils.logDuration("ElevClArmSubsystem.printDashboard", () -> {
      SmartDashboard.putString("ElevArm State:", state.toString());
      SmartDashboard.putString("Requested State:", requestState.toString());

      SmartDashboard.putString("Control Mode:", getEMode().toString());
      SmartDashboard.putString("Requested Mode:", requestMode.toString());
      SmartDashboard.putString("Claw State:", clawstate.toString());
      // SmartDashboard.putBoolean("Want to Shoot:", shootLust);

      SmartDashboard.putBoolean("Coral in Hopper:", isCoralInHopper());
      SmartDashboard.putBoolean("Coral in Claw:", isCoralEnteredClaw());
      SmartDashboard.putBoolean("Coral Ready to Go:", isCoralSkibidi());

      SmartDashboard.putNumber("Elevator current:", rightElevatorMotor.getCurrent());
      SmartDashboard.putNumber("Arm current:", shoulderMotor.getCurrent());
      SmartDashboard.putNumber("Claw current:", clawMotor.getCurrent());

      // SmartDashboard.putNumber("Elevator voltage applied:",
      //     rightElevatorMotor.motor.getMotorVoltage().getValue().magnitude());
      // SmartDashboard.putNumber("Arm voltage applied:", shoulderMotor.motor.getMotorVoltage().getValue().magnitude());
      // SmartDashboard.putNumber("Claw voltage applied:", clawMotor.motor.getMotorVoltage().getValue().magnitude());

      // SmartDashboard.putNumber("Claw Start Pos:", clawStartPosition);
      // SmartDashboard.putNumber("Claw Target Pos:", clawTargetPosition);
      // SmartDashboard.putBoolean("Position Control:", positionControl);

      // leftElevatorMotor.putPIDF();
      // rightElevatorMotor.putPIDF();
      // shoulderMotor.putPIDF();
      // clawMotor.putPIDF();

      // leftElevatorMotor.putPV();
      rightElevatorMotor.putP();
      shoulderMotor.putP();
      // clawMotor.putPV();
    });

  }

  public void setClawStartPosition() {
    clawStartPosition = clawMotor.getPosition();
  }
}
