package frc.robot.commands;

import frc.robot.ExtraMath;
import frc.robot.OURLimelightHelpers;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.input.SnapButton;
import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem.ControlMode;
import frc.robot.subsystems.ElevClArmSubsystem.RequestState;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class LimelightCmd extends Command {
  private final ElevClArmSubsystem e;
  private final CommandSwerveDrivetrain drive;

  private final SnapButton snap;
  public final RequestState state;
  private final double ep;
  Timer shootTimer;
  private boolean finished = false;

  private double positionError;

  private double overrideX, overrideY, overrideRot = 0.0;

  public LimelightCmd(ElevClArmSubsystem e, CommandSwerveDrivetrain drivetrain, SnapButton snap, RequestState state) {
    this(e, drivetrain, snap, state, 0.25);
  }

  public LimelightCmd(ElevClArmSubsystem e, CommandSwerveDrivetrain drivetrain, SnapButton snap, RequestState state, double epsilon) {
    this.e = e;
    this.drive = drivetrain;
    this.snap = snap;
    this.state = state;
    this.ep = epsilon;
    shootTimer = new Timer();

    addRequirements(e);
  }

  @Override
  public void initialize() {
    e.requestState(state);
    shootTimer.stop();
    shootTimer.reset();
    finished = false;

    positionError = Double.MAX_VALUE; // update me later
    // Override the X feedback
    PPHolonomicDriveController.overrideXFeedback(() -> {
      // Calculate feedback from your custom PID controller
      return overrideX;
    });

    PPHolonomicDriveController.overrideYFeedback(() -> {
      // Calculate feedback from your custom PID controller
      return overrideY;
    });
    PPHolonomicDriveController.overrideRotationFeedback(() -> {
      // Calculate feedback from your custom PID controller
      return overrideRot;
    });
  }

  @Override
  public void execute() {

    // e movement stuff
    if (e.atPosition(ep) && state.finaleState() == e.state && shootTimer.get() == 0) {
      e.shootLust = true;
      shootTimer.restart();
    }; 
    //TODO adjust the value
    if (shootTimer.get() > 0.3) {
      e.requestState(RequestState.None);
      e.shootLust = false;
    }

    if (shootTimer.get() > 0.8) {
      finished = true; // blame Adam for this brain death
    }

    // limelight snaps
    if (snap == SnapButton.Right || snap == SnapButton.Left || snap == SnapButton.Center) {
      double tx, ty, yaw;
      double targetTx, targetTy = 0.58, targetYaw = 0; // define unchanging values
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

      camRet = OURLimelightHelpers.getValidBotPose(primaryCam, fallbackCam, null);
      if (camRet != null) {
        botPose = camRet[0]; // TODO lock ID
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

        double overrideXRC = ExtraMath.clampedDeadzone(vectorY * -pt, 1, .03);
        double overrideYRC = ExtraMath.clampedDeadzone(vectorX * -pt, 1, .03);
        overrideRot = ExtraMath.clampedDeadzone( vectorYaw * -pr, 1, .1);

        double thetaRadians = drive.getState().Pose.getRotation().getRadians();
        // Apply the 2D rotation formula
        overrideX = overrideXRC * Math.cos(thetaRadians) - overrideYRC * Math.sin(thetaRadians);
        overrideY = overrideXRC * Math.sin(thetaRadians) + overrideYRC * Math.cos(thetaRadians);
        SmartDashboard.putNumber("Position Error", positionError);
       

        
        // Y goes in X and X goes in y because of 
        // setDefaultCommand
        // drive.applyRequest(() -> {
        //   return driveRC.withVelocityX(ExtraMath.clampedDeadzone(vectorY * -pt, 1, .03)) // Drive
        //       .withVelocityY(ExtraMath.clampedDeadzone(vectorX * -pt, 1, .03))
        //       .withRotationalRate(ExtraMath.clampedDeadzone(vectorYaw * -pr, 1, .1));

        // });
      }
      else{
        overrideX = 0;
        overrideY = 0;
        overrideRot = 0;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Clear all feedback overrides
    PPHolonomicDriveController.clearFeedbackOverrides();
    // still stow if interrupted
    e.requestState(RequestState.None);
    e.shootLust = false;
  }

  @Override
  public boolean isFinished() {
    return finished;
  }

}
