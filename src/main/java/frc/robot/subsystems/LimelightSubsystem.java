package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase implements Runnable {
  public boolean isLinedUp = false;

  String name;
  public double tagTx = 0.0;
  public double tagTy = 0.0;
  public boolean tagTv = false;

  public LimelightSubsystem(String name) {
    this.name = name;
    new Thread(this, "`" + name + "` Thread").start();
  }

  @Override
  public void periodic() {
    printDashboard();
  }

  String dashboardKey() {
    return "`" + name + "`";
  }

  // Prints various values of every target
  public void printDashboard() {
    synchronized (this) {
      SmartDashboard.putNumber(dashboardKey() + "TX", tagTx);
      SmartDashboard.putNumber(dashboardKey() + "TY", tagTy);
      // SmartDashboard.putBoolean(dashboardKey() + "TV", tagTv);
      // SmartDashboard.putBoolean(dashboardKey() + "Rotation Enabled", limelightRotation);
    }
  }

  public void setPipeline(int pipeline) {
    LimelightHelpers.setPipelineIndex(name, pipeline);
  }

  @Override
  public void run() {
    while (true) {
      synchronized (this) {
        tagTx = LimelightHelpers.getTX(name);
        tagTy = LimelightHelpers.getTY(name);
        tagTv = LimelightHelpers.getTV(name);
      }
      try {
        Thread.sleep(10);
      } catch (InterruptedException iex) {
      }
    }
  }
}
