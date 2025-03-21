package frc.robot.commands.auto;

import frc.robot.ExtraMath;
import frc.robot.LimelightHelpers;
import frc.robot.OURLimelightHelpers;
import frc.robot.ReefSide;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.input.SnapButton;
import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem.RequestState;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class LimelightAutoCmd extends Command {
  private final ElevClArmSubsystem e;
  private final CommandSwerveDrivetrain drive;
  private final SwerveRequest.RobotCentric swerve;

  private final SnapButton snap;
  public final RequestState state;
  private final double ep;
  Timer shootTimer;
  Timer abandonTimer;
  PoseEstimate llMeasurementTemp;
  private boolean finished = false;

  private double positionError;

  double idToLookFor;
    boolean driveAtPosition;
  ReefSide reefSide;
  
    public LimelightAutoCmd(ReefSide reefSide, ElevClArmSubsystem e, CommandSwerveDrivetrain drivetrain, SnapButton snap,
        RequestState state, SwerveRequest.RobotCentric swerve) {
      this(reefSide, e, drivetrain, snap, state, swerve, 0.25);
    }
  
    public LimelightAutoCmd(ReefSide reefSide, ElevClArmSubsystem e, CommandSwerveDrivetrain drivetrain, SnapButton snap,
        RequestState state, SwerveRequest.RobotCentric swerve, double epsilon) {
      this.e = e;
      this.drive = drivetrain;
      this.snap = snap;
      this.state = state;
      this.ep = epsilon;
      this.swerve = swerve;
      this.reefSide = reefSide;
      shootTimer = new Timer();


      abandonTimer = new Timer();
  
      addRequirements(e, drivetrain); // TODO should this be required? We do command the drivetrain...
    }
  
    @Override
    public void initialize() {
      driveAtPosition = false;
      shootTimer.stop();
      shootTimer.reset();
      abandonTimer.stop();
      abandonTimer.reset();
      finished = false;
      idToLookFor = reefSide.getTag();

    positionError = Double.MAX_VALUE; // update me later
    e.requestState(state);

  }

  @Override
  public void execute() {

    // e movement stuff
    if (e.atFinalPosition(ep) && shootTimer.get() == 0 && driveAtPosition) {
      e.shootLust = true;
      shootTimer.restart();
    }
    if (e.atFinalPosition(ep) && abandonTimer.get() == 0) {
      abandonTimer.restart();
    }
    // TODO adjust the value
    if (abandonTimer.get() > 1.0) {
      e.shootLust = true;
    }
    if (shootTimer.get() > 0.3 || abandonTimer.get() > 1.5) {
      e.shootLust = false;
      e.requestState(RequestState.None);
    }

    if (shootTimer.get() > 0.4 || abandonTimer.get() > 2.0) {
      finished = true; // blame Adam for this brain death
    }

    // limelight snaps
    if (snap == SnapButton.Right || snap == SnapButton.Left || snap == SnapButton.Center) {
      double tx, ty, yaw;
      double targetTx, targetTy = 0.587, targetYaw = 0; // define unchanging values
      double[][] camRet;
      double[] botPose = null;

      String primaryCam, fallbackCam;

      switch (snap) {
        case Right:
          primaryCam = "limelight-left";
          fallbackCam = "limelight-right";
          // targetTy = 0.587;
          // targetYaw = 0;
          targetTx = 0.170;
          if(idToLookFor == 6 || idToLookFor == 19){
            targetTx = 0.130;
          }
          if(idToLookFor == 9 || idToLookFor == 22){
            targetTy = 0.6;
          }
          if(idToLookFor == 20 || idToLookFor == 11){
            targetTy = 0.56;
          }
          break;
        case Left:
          primaryCam = "limelight-right";
          fallbackCam = "limelight-left";
          targetTx = -0.17;
          if(idToLookFor == 6 || idToLookFor == 19){
            targetTx = -0.22;
          }
          if(idToLookFor == 9 || idToLookFor == 22){
            targetTx = -0.22;
          }
          break;
        default:
          primaryCam = "limelight-left";
          fallbackCam = "limelight-right";
          targetTx = 0.0;
          targetTy = 0.5;
      }

      camRet = OURLimelightHelpers.getBotPoseTargetSpace(primaryCam, fallbackCam, idToLookFor, 10000000.0);
      SmartDashboard.putNumber("idlooking", idToLookFor);

      if (camRet != null) {
        botPose = camRet[0];
      }

      double brainYRC;
      double brainXRC;
      double brainRot;
      if (botPose != null && botPose.length >= 5) {
        tx = botPose[0]; // meters
        ty = -botPose[2]; // meters - secretly grabbing tz - away is
                          // more negative
        yaw = botPose[4]; // degrees

        // vector's represent needed movement from the robot to the tag
        // in targetspace
        double vectorX = targetTx - tx;
        double vectorY = targetTy - ty;
        double vectorYaw = targetYaw - yaw;
        positionError = Math.sqrt((vectorX * vectorX) + (vectorY * vectorY));

        double pt = 2.5; // translation p value
        double pr = 0.1; // rotation p

        brainXRC = ExtraMath.clampedDeadzone(vectorY * -pt, 0.6, .03);
        brainYRC = ExtraMath.clampedDeadzone(vectorX * -pt, 0.6, .03);
        brainRot = ExtraMath.clampedDeadzone(vectorYaw * -pr, 1, .1);

        if (Math.abs(positionError) < 0.045) {
          driveAtPosition = true;

          // Reset odometry when we successfully align with the reef
          // var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(primaryCam);
          // if (llMeasurement != null) {
          //   drive.resetPose(llMeasurement.pose);
          // }
        }

      } else {
        brainXRC = brainYRC = brainRot = 0;
      }
      double thetaRadians = drive.getState().Pose.getRotation().getRadians();
      SmartDashboard.putNumber("Position Error", positionError);
      SmartDashboard.putNumber("Velocity X", brainXRC);
      SmartDashboard.putNumber("Velocity Y", brainYRC);
      SmartDashboard.putNumber("Rotation", thetaRadians);
      drive.setControl(swerve.withVelocityX(brainXRC)
          .withVelocityY(brainYRC)
          .withRotationalRate(brainRot));
    }
  }

  @Override
  public void end(boolean interrupted) {
    // still stow if interrupted
    e.requestState(RequestState.None);
    e.shootLust = false;
    int[] ids = {};
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-left", ids);
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-right", ids);
  }

  @Override
  public boolean isFinished() {
    return finished;
  }

}
