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

public class AlgaeAutoCmd extends Command {
  private final ElevClArmSubsystem e;
  private final CommandSwerveDrivetrain drive;
  private final SwerveRequest.RobotCentric swerve;

  public final RequestState state;
  private final double ep;
  Timer pickupTimer;
  Timer abandonTimer;
  PoseEstimate llMeasurementTemp;
  private boolean finished = false;

  private double positionError;

  double idToLookFor;
  boolean driveAtPosition;
  ReefSide reefSide;

  public AlgaeAutoCmd(ReefSide reefSide, ElevClArmSubsystem e, CommandSwerveDrivetrain drivetrain,
      RequestState state, SwerveRequest.RobotCentric swerve) {
    this(reefSide, e, drivetrain, state, swerve, 0.25);
  }

  public AlgaeAutoCmd(ReefSide reefSide, ElevClArmSubsystem e, CommandSwerveDrivetrain drivetrain,
      RequestState state, SwerveRequest.RobotCentric swerve, double epsilon) {
    this.e = e;
    this.drive = drivetrain;
    this.state = state;
    this.ep = epsilon;
    this.swerve = swerve;
    this.reefSide = reefSide;
    pickupTimer = new Timer();
    abandonTimer = new Timer();

    addRequirements(e, drivetrain); // TODO should this be required? We do command the drivetrain...
  }

  @Override
  public void initialize() {
    driveAtPosition = false;
    pickupTimer.stop();
    pickupTimer.reset();
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
    if (e.atFinalPosition(ep) && pickupTimer.get() == 0 && driveAtPosition) {
      pickupTimer.restart();
    }
    if (pickupTimer.get() > 1) {
      e.requestState(RequestState.None);
    }

    // if (pickupTimer.get() > 3) {
    // finished = true; // blame Adam for this brain death
    // }

    double tx, ty, yaw;
    double targetTx, targetTy = 0.587, targetYaw = 0; // define unchanging values
    double[][] camRet;
    double[] botPose = null;

    String primaryCam, fallbackCam;

    primaryCam = "limelight-left"; // center snap
    fallbackCam = "limelight-right";
    targetTx = 0.0;
    targetTy = 0.5;

    camRet = OURLimelightHelpers.getBotPoseTargetSpace(primaryCam, fallbackCam, idToLookFor, 10000000.0);
    SmartDashboard.putNumber("idlooking", idToLookFor);

    if (camRet != null) {
      botPose = camRet[0];
    }

    double brainYRC;
    double brainXRC;
    double brainRot;
    if (botPose != null) {
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

      brainXRC = ExtraMath.clampedDeadzone(vectorY * -pt, 0.4, .03);
      brainYRC = ExtraMath.clampedDeadzone(vectorX * -pt, 0.4, .03);
      brainRot = ExtraMath.clampedDeadzone(vectorYaw * -pr, 1, .1);

      if (Math.abs(positionError) < 0.045) {
        driveAtPosition = true;
      }

    } else {
      brainXRC = brainYRC = brainRot = 0;
    }
    double thetaRadians = drive.getState().Pose.getRotation().getRadians();
    SmartDashboard.putNumber("Position Error", positionError);
    SmartDashboard.putNumber("Velocity X", brainXRC);
    SmartDashboard.putNumber("Velocity Y", brainYRC);
    SmartDashboard.putNumber("Rotation", thetaRadians);
    if (pickupTimer.get() > 1 && pickupTimer.get() < 2) {
      drive.setControl(swerve.withVelocityX(-1.5)
          .withVelocityY(0)
          .withRotationalRate(0));
    } else if (pickupTimer.get() > 2) {
      finished = true;
      drive.setControl(swerve.withVelocityX(0)
          .withVelocityY(0)
          .withRotationalRate(0));
    } else {
      drive.setControl(swerve.withVelocityX(brainXRC)
          .withVelocityY(brainYRC)
          .withRotationalRate(brainRot));
    }

  }

  @Override
  public void end(boolean interrupted) {
    // still stow if interrupted
    e.requestState(RequestState.None);
    int[] ids = {};
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-left", ids);
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-right", ids);
  }

  @Override
  public boolean isFinished() {
    return finished;
  }

}
