package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.Pigeon2;

public class PigeonSubsystem extends SubsystemBase {
  Pigeon2 pigeon;
  public double X, Y, Z;
  public double accelX, accelY, accelZ;
  public double temp;
  public boolean rotateToDegree = false;
  public double magnitudeToAngle = 0;
  public double targetAngle = 0;

  public PigeonSubsystem() {
    pigeon = new Pigeon2(Constants.PIGEON_ID,"DriveTrain");
  }

  @Override
  public void periodic() {
    try {
      updateValues();
      printDashboard();
    } catch (Exception e) {
      // SmartDashboard.putString("CTRE Last Error", e.getMessage());
    }
  }

  public void updateValues() {
    X = pigeon.getPitch().getValueAsDouble();
    Y  = pigeon.getYaw().getValueAsDouble();
    Z = pigeon.getRoll().getValueAsDouble();
    accelX = pigeon.getAccelerationX().getValueAsDouble();
    accelY = pigeon.getAccelerationY().getValueAsDouble();
    accelZ = pigeon.getAccelerationZ().getValueAsDouble();
    temp = pigeon.getTemperature().getValueAsDouble();
  }

  public void printDashboard() {
    // SmartDashboard.putNumber("Pigeon X", X);
    SmartDashboard.putNumber("Pigeon Y", Y);
    // SmartDashboard.putNumber("Pigeon Z", Z);
    // SmartDashboard.putNumber("Pigeon Acc X", accelX);
    // SmartDashboard.putNumber("Pigeon Acc Y", accelY);
    // SmartDashboard.putNumber("Pigeon Acc Z", accelZ);
    // SmartDashboard.putNumber("Pigeon Current Temp [C]", temp);
  }

  public double getRotationAngleFromZero(){
    return pigeon.getYaw().getValueAsDouble()%360;
  }

  // don't do this
//   public void zeroYaw(){
//     var isRedAlliance = Constants.alliance == Alliance.Red;
//     if(isRedAlliance){
//       pigeon.setYaw(180);
//     }
//     else{
//       pigeon.setYaw(0);
//     }
//   }
}