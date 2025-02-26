package frc.robot.commands;

import frc.robot.ExtraMath;
import frc.robot.LimelightHelpers;
import frc.robot.OURLimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.input.SnapButton;
import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem.RequestState;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
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
  Pose2d llMeasurement;
  PoseEstimate llMeasurementTemp;
  boolean stopRequesting = false;
  private boolean finished = false;

  private double positionError;

  // private double overrideX, overrideY, overrideRot = 0.0;

  Double idToLookFor;
  Boolean driveAtPosition;

  public LimelightAutoCmd(ElevClArmSubsystem e, CommandSwerveDrivetrain drivetrain, SnapButton snap,
  RequestState state, SwerveRequest.RobotCentric swerve) {
    this(e, drivetrain, snap, state, swerve, 0.25);
  }

  public LimelightAutoCmd(ElevClArmSubsystem e, CommandSwerveDrivetrain drivetrain, SnapButton snap,
      RequestState state, SwerveRequest.RobotCentric swerve,
      double epsilon) {
    this.e = e;
    this.drive = drivetrain;
    this.snap = snap;
    this.state = state;
    this.ep = epsilon;
    this.swerve = swerve;
    shootTimer = new Timer();
    abandonTimer = new Timer();
    llMeasurement = null;

    addRequirements(e, drivetrain);
  }

  @Override
  public void initialize() {
    driveAtPosition = false;
    shootTimer.stop();
    shootTimer.reset();
    abandonTimer.stop();
    abandonTimer.reset();
    finished = false;
    idToLookFor = null;
    e.setClawStartPosition();
    e.positionControl = true;
    stopRequesting = false;
    llMeasurement = null;

    positionError = Double.MAX_VALUE; // update me later
    // Override the X feedback
    // PPHolonomicDriveController.overrideXFeedback(() -> {
    // // Calculate feedback from your custom PID controller
    // return overrideX;
    // });

    // PPHolonomicDriveController.overrideYFeedback(() -> {
    // // Calculate feedback from your custom PID controller
    // return overrideY;
    // });
    // PPHolonomicDriveController.overrideRotationFeedback(() -> {
    // // Calculate feedback from your custom PID controller
    // return overrideRot;
    // });
  }

  @Override
  public void execute() {

    // e movement stuff
    if (e.atPosition(ep) && state.finaleState() == e.state && shootTimer.get() == 0 && driveAtPosition == true) {
      e.shootLust = true;
      shootTimer.restart();
    }
    if (e.atPosition(ep) && state.finaleState() == e.state && abandonTimer.get() == 0) {
      abandonTimer.restart();
    }
    // TODO adjust the value
    if (abandonTimer.get() > 1.0) {
      e.shootLust = true;
    }
    if (shootTimer.get() > 0.3 || abandonTimer.get() > 1.5) {
      stopRequesting = true;
      e.shootLust = false;
      e.requestState(RequestState.None);
      if (snap == SnapButton.Right) {
        llMeasurementTemp = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
        if (llMeasurementTemp != null) {
          llMeasurement = llMeasurementTemp.pose;
        }
      }
      if (snap == SnapButton.Left) {
        llMeasurementTemp = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-Right");
        if (llMeasurementTemp != null) {
          llMeasurement = llMeasurementTemp.pose;
        }
      }
    }

    if (shootTimer.get() > 0.8 || abandonTimer.get() > 2.0) {
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
          targetTx = 0.150;
          // targetTy = 0.587;
          // targetYaw = 0;
          break;
        case Left:
          primaryCam = "limelight-right";
          fallbackCam = "limelight-left";
          targetTx = -0.18;
          break;
        default:
          primaryCam = "limelight-left";
          fallbackCam = "limelight-right";
          targetTx = 0.0;
          targetTy = 0.5;
      }

      camRet = OURLimelightHelpers.getBotPoseTargetSpace(primaryCam, fallbackCam, idToLookFor, 10000000.0);
      if (camRet != null) {
        idToLookFor = camRet[1][0];
        botPose = camRet[0];
        SmartDashboard.putNumber("idlooking", idToLookFor);
      }

      if (idToLookFor != null) {
        if (!stopRequesting) {
          e.requestState(state);
        }
      }

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

        double brainXRC = ExtraMath.clampedDeadzone(vectorY * -pt, 0.4, .03);
        double brainYRC = ExtraMath.clampedDeadzone(vectorX * -pt, 0.4, .03);
        double brainRot = ExtraMath.clampedDeadzone(vectorYaw * -pr, 1, .1);

        if (Math.abs(positionError) < 0.045) {
          driveAtPosition = true;
        }

        double thetaRadians = drive.getState().Pose.getRotation().getRadians();
        // // Apply the 2D rotation formula
        // overrideX = overrideXRC * Math.cos(thetaRadians) - overrideYRC *
        // Math.sin(thetaRadians);
        // overrideY = overrideXRC * Math.sin(thetaRadians) + overrideYRC *
        // Math.cos(thetaRadians);
        SmartDashboard.putNumber("Position Error", positionError);
        SmartDashboard.putNumber("Velocity X", brainXRC);
        SmartDashboard.putNumber("Velocity Y", brainYRC);
        SmartDashboard.putNumber("Rotation", thetaRadians);
        // Y goes in X and X goes in y because of
        // setDefaultCommand
        drive.applyRequest(() -> {
          return swerve.withVelocityX(brainYRC) // Drive
              .withVelocityY(brainXRC)
              .withRotationalRate(brainRot);
        }).schedule(); // TODO: not sure if this is the right way to do it...

      } else {
        drive.applyRequest(() -> {
          return swerve.withVelocityX(0) // Drive
              .withVelocityY(0)
              .withRotationalRate(0);
        }).schedule();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Clear all feedback overrides
    // if(llMeasurement != null){
    // drive.resetPose(llMeasurement);
    // }

    // PPHolonomicDriveController.clearFeedbackOverrides();
    // still stow if interrupted
    // e.requestState(RequestState.None);
    // e.shootLust = false;
    int[] ids = {};
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-left", ids);
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-right", ids);
  }

  @Override
  public boolean isFinished() {
    return finished;
  }

}
