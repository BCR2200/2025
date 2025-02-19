package frc.robot.commands;

import frc.robot.ExtraMath;
import frc.robot.OURLimelightHelpers;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.input.SnapButton;
import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem.ControlMode;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.wpilibj2.command.Command;

public class LimelightCmd extends Command {
  private final ElevClArmSubsystem e;
  private final CommandSwerveDrivetrain drive;

  private final SnapButton snap;
  private final RobotCentric driveRC;
  private double positionError;

  public LimelightCmd(ElevClArmSubsystem e, CommandSwerveDrivetrain drivetrain, SnapButton snap, RobotCentric driveRC) {
    this.e = e;
    this.drive = drivetrain;
    this.snap = snap;
    this.driveRC = driveRC;
    addRequirements(drive, e);
  }

  @Override
  public void initialize() {
    positionError = Double.MAX_VALUE; // update me later
  }

  @Override
  public void execute() {

    // limelight snaps
    if (snap == SnapButton.Right || snap == SnapButton.Left || snap == SnapButton.Center) {
      double tx, ty, yaw;
      double targetTx, targetTy = 0.587, targetYaw = 0; // define unchanging values
      double[] botPose;

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

      botPose = OURLimelightHelpers.getValidBotPose(primaryCam, fallbackCam);

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
        positionError = Math.sqrt(vectorX * vectorX + vectorY * vectorY + vectorYaw * vectorYaw);

        double pt = 2.5; // translation p value
        double pr = 0.1; // rotation p

        // Y goes in X and X goes in y because of comment above
        // setDefaultCommand
        drive.applyRequest(() -> {
          return driveRC.withVelocityX(ExtraMath.clampedDeadzone(vectorY * -pt, 1, .03)) // Drive
              .withVelocityY(ExtraMath.clampedDeadzone(vectorX * -pt, 1, .03))
              .withRotationalRate(ExtraMath.clampedDeadzone(vectorYaw * -pr, 1, .1));

        });
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return positionError < 0.03;
  }

}
