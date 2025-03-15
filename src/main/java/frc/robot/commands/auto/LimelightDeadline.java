package frc.robot.commands.auto;

import frc.robot.LimelightHelpers;
import frc.robot.OURLimelightHelpers;
import frc.robot.ReefSide;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.input.SnapButton;
import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem.RequestState;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class LimelightDeadline extends Command {

  private final SnapButton snap;
  private final double distance;
  private boolean finished = false;

  double idToLookFor;
  ReefSide reefSide;

  public LimelightDeadline(ReefSide reefSide, SnapButton snap) {
    this(reefSide, snap, 1.5);
  }

  public LimelightDeadline(ReefSide reefSide, SnapButton snap, double distance) {
    this.snap = snap;
    this.distance = distance;
    this.reefSide = reefSide;
  }

  @Override
  public void initialize() {
    idToLookFor = reefSide.getTag();
  }

  @Override
  public void execute() {

    // limelight snaps
    if (snap == SnapButton.Right || snap == SnapButton.Left || snap == SnapButton.Center) {

      String primaryCam, fallbackCam;

      switch (snap) {
        case Right:
          primaryCam = "limelight-left";
          fallbackCam = "limelight-right";
          break;
        case Left:
          primaryCam = "limelight-right";
          fallbackCam = "limelight-left";
          break;
        default:
          primaryCam = "limelight-left";
          fallbackCam = "limelight-right";
      }

      finished = OURLimelightHelpers.isCloseEnough(primaryCam, fallbackCam, idToLookFor, distance);
      SmartDashboard.putNumber("idlooking", idToLookFor);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // still stow if interrupted
    int[] ids = {};
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-left", ids);
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-right", ids);
  }

  @Override
  public boolean isFinished() {
    return finished;
  }

}
