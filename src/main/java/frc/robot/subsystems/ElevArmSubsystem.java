package frc.robot.subsystems;

import java.io.UncheckedIOException;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDMotor;

public class ElevArmSubsystem extends SubsystemBase {
  // Elevator x 2 kraken
  // 13.1 revolutions of elevator = 7.85 inches of travel
  // Shoulder x 1 kraken
  // 73.5 revolutions = 360 degrees (all the way around, not good)
  // Contains beam break
  public PIDMotor leftElevatorMotor;
  public PIDMotor rightElevatorMotor;
  public PIDMotor shoulderMotor;

  public DigitalInput hopperBreak;

  public Zone zone1 = new Zone(1, new ElevArmPosition(1,1), new ElevArmPosition(1,1), new ElevArmPosition(1,1));
  public Zone zone2 = new Zone(2, new ElevArmPosition(1,1), new ElevArmPosition(1,1), new ElevArmPosition(1,1));
  public Zone zone3 = new Zone(3, new ElevArmPosition(1,1), new ElevArmPosition(1,1), new ElevArmPosition(1,1));
  public Zone zone4 = new Zone(4, new ElevArmPosition(1,1), new ElevArmPosition(1,1), new ElevArmPosition(1,1));
  public Zone zone5 = new Zone(5, new ElevArmPosition(1,1), new ElevArmPosition(1,1), new ElevArmPosition(1,1));
  

  public class ElevArmPosition {
    public double elevatorPos;
    public double armPos;

    public ElevArmPosition(double elevatorPos, double armPos){
      this.elevatorPos = elevatorPos;
      this.armPos = armPos;
    }
  }

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
      return (position.elevatorPos < max.elevatorPos && position.elevatorPos > min.elevatorPos) && (position.armPos < max.armPos && position.armPos > min.armPos);
    }
  }

  public enum ArmState {
    Up, Down, Left, Right
  }

  public ElevArmSubsystem() {
    leftElevatorMotor = PIDMotor.makeMotor(Constants.LEFT_ELEVATOR_ID, "left elevator", 0, 0, 0, 0, 0, 0, 0, 0, 0);
    rightElevatorMotor = PIDMotor.makeMotor(Constants.RIGHT_ELEVATOR_ID, "right elevator", 0, 0, 0, 0, 0, 0, 0, 0, 0);
    shoulderMotor = PIDMotor.makeMotor(Constants.SHOULDER_ID, "shoulder", 0, 0, 0, 0, 0, 0, 0, 0, 0);
    leftElevatorMotor.follow(rightElevatorMotor, true);

    leftElevatorMotor.setCurrentLimit(30);
    rightElevatorMotor.setCurrentLimit(30);
    shoulderMotor.setCurrentLimit(30);

    hopperBreak = new DigitalInput(Constants.HOPPER_ID);

  }

  @Override
  public void periodic() {

    // if (zeroing) {
    // leftShoulderMotor.setPercentOutput(0.05);
    // percentOutZeroCheck = true;
    // } else if (percentOutZeroCheck) {
    // leftShoulderMotor.setPercentOutput(0.0);
    // percentOutZeroCheck = false;
    // leftShoulderMotor.resetEncoder();
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
    // SmartDashboard.putString("Claw State:", state.toString());
    // clawMotor.putPIDF();
    // clawMotor.putPV();
  }
}
