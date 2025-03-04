package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.timing.TimingUtils;

public class PigeonSubsystem extends SubsystemBase {
  Pigeon2 pigeon;
  public double X, Y, Z;
  public double accelX, accelY, accelZ;
  public double temp;

  public PigeonSubsystem() {
    pigeon = new Pigeon2(Constants.PIGEON_ID,"DriveTrain");
  }

  @Override
  public void periodic() {
    TimingUtils.logDuration("PigeonSubsystem.periodic", () -> {
      try {
        updateValues();
        // printDashboard();
      } catch (Exception e) {
        // SmartDashboard.putString("CTRE Last Error", e.getMessage());
      }
    });
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
    SmartDashboard.putNumber("Pigeon X", X);
    SmartDashboard.putNumber("Pigeon Y", Y);
    SmartDashboard.putNumber("Pigeon Z", Z);
    SmartDashboard.putNumber("Pigeon Acc X", accelX);
    SmartDashboard.putNumber("Pigeon Acc Y", accelY);
    SmartDashboard.putNumber("Pigeon Acc Z", accelZ);
    SmartDashboard.putNumber("Pigeon Current Temp [C]", temp);
  }
}