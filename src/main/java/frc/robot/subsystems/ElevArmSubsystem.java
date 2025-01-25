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

  public Zone zone1 = new Zone(1, 0, 1, 0, 1, 0.5,0.5);
  public Zone zone2 = new Zone(2, 0, 1, 0, 1, 0.5, 0.5);
  public Zone zone3 = new Zone(3, 0, 1, 0, 1, 0.5, 0.5);
  public Zone zone4 = new Zone(4, 0, 1, 0, 1, 0.5, 0.5);
  public Zone zone5 = new Zone(5, 0, 1, 0, 1, 0.5, 0.5);

  public class Zone {
    public int zoneIndex;
    public double elevatorMin;
    public double elevatorMax;
    public double armMin;
    public double armMax;
    public double safeElevatorPos;
    public double safeArmPos;
    
    //adjacent

    public Zone(int zone, double eMin, double eMax, double aMin, double aMax, double safeE, double safeA) {
      if(eMax < eMin){
        throw new UncheckedIOException("Elevator max is less than min", null);
      }
      if(aMax < aMin){
        throw new UncheckedIOException("Arm max is less than min", null);
      }
      if(safeE > eMax || safeE < eMin){
        throw new UncheckedIOException("Elevator safe position is greater than max or less than min", null);
      }
      if(safeA > aMax || safeA < aMin){
        throw new UncheckedIOException("Arm safe position is greater than max or less than min", null);
      }

      zoneIndex = zone;
      elevatorMin = eMin;
      elevatorMax = eMax;
      armMin = aMin;
      armMax = aMax;
      safeE = safeElevatorPos;
      safeA = safeArmPos;
    }

    public boolean isItIn(double elevatorPos, double armPos){
      return (elevatorPos < elevatorMax && elevatorPos > elevatorMin) && (armPos < armMax && armPos > armMin);
    }

    public double safeElevator(){
      return safeElevatorPos;
    }

    public double safeArm(){
      return safeArmPos;
    }
  }

  public enum ArmState {
    Up,Down,Left,Right
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
    //     leftShoulderMotor.setPercentOutput(0.05);
    //     percentOutZeroCheck = true;
    // } else if (percentOutZeroCheck) {
    //     leftShoulderMotor.setPercentOutput(0.0);
    //     percentOutZeroCheck = false;
    //     leftShoulderMotor.resetEncoder();
    // }
  } 

  //return next elevator/arm
  public void go(double currentElevator, double currentArm, double goalElevator, double goalArm) {
    
    // idk how else to find the zone
    Zone[] zones = { zone1, zone2, zone3, zone4, zone5 };
    Zone currentZone = null;
    for (Zone zone : zones) {
        if (zone.isItIn(currentElevator, currentArm)) {
            currentZone = zone;
            break;
        }
    }

    Zone targetZone = null;
    for (Zone zone : zones) {
        if (zone.isItIn(goalElevator, goalArm)) {
            targetZone = zone;
            break;
        }
    }

    if (currentZone == null || targetZoneIndex < 1 || targetZoneIndex > zones.length) {
        System.out.println("What are you doing? Target isn't in a zone??");
        return;
    }

    int currentZoneIndex = currentZone.zoneIndex;
    
      while (currentZoneIndex != targetZoneIndex) {
          
          int nextZoneIndex = currentZoneIndex < targetZoneIndex ? currentZoneIndex + 1 : currentZoneIndex - 1;
          Zone nextZone = zones[nextZoneIndex - 1];
  
          // go to triangle spots
          rightElevatorMotor.setTarget(nextZone.safeElevator());
          shoulderMotor.setTarget(nextZone.safeArm());
  
          currentZoneIndex = nextZoneIndex;
  
          // wait for movement
      }

    // in target zone, move to direct
    rightElevatorMotor.setTarget(goalElevator); 
    shoulderMotor.setTarget(goalArm); 
}


  //manage positions asked to, only go if safe

  public void printDashboard() {
    // SmartDashboard.putString("Claw State:", state.toString());
    // clawMotor.putPIDF();
    // clawMotor.putPV();
  }
}
