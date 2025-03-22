
package frc.robot;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.commands.auto.AutoCommand;
import frc.robot.commands.auto.WarmupAutoCmd;
import frc.robot.timing.TimingUtils;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  final Field2d m_field = new Field2d();

  double lastDashboardUpdate = 0;

  public static Alliance alliance = Alliance.Blue;

  public static final boolean isCompBot = false;

  public Robot() {
    m_robotContainer = new RobotContainer();
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    updateAlliance();
    m_robotContainer.updateDrivetrainRobotPerspective();

  }

  public static void updateAlliance() {
    alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  @Override
  public void robotInit() {

    m_robotContainer.autoChooser.onChange(this::updateFieldPaths);
    updateAlliance();
  }

  public void updateFieldPaths(AutoCommand auto) {
    if (auto != null) {
      m_field.getObject("path").setPoses(auto.getAllProperFlippedPathPoses());
    } else {
      m_field.getObject("path").setPoses();
    }
  }

  public final String[] limelights = { "limelight-left", "limelight-right" };

  @Override
  public void robotPeriodic() {
    TimingUtils.logDuration("robotPeriodic", () -> {
      TimingUtils.logDuration("commandScheduler", () -> {
        CommandScheduler.getInstance().run();
      });
      if (Timer.getFPGATimestamp() > lastDashboardUpdate + 0.200) {
        SmartDashboard.putData("Field", m_field);

        lastDashboardUpdate = Timer.getFPGATimestamp();
        var driveState = m_robotContainer.drivetrain.getState();
        m_field.setRobotPose(driveState.Pose);
        SmartDashboard.putBoolean("RobotThinksItIsOnRed", alliance == Alliance.Red);

        if (m_robotContainer.driverController.getHID().getBackButtonPressed()) {
          updateAlliance();
          updateFieldPaths(m_robotContainer.autoChooser.getSelected());
        }
      }

      var botState = m_robotContainer.drivetrain.getState();
      double omegarps = Units.radiansToRotations(botState.Speeds.omegaRadiansPerSecond);

      if (DriverStation.isEnabled()) {
        for (int i = 0; i < limelights.length; ++i) {
          LimelightHelpers.SetRobotOrientation(limelights[i], botState.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
          PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelights[i]);
          if (poseEstimate != null && poseEstimate.tagCount > 0 && Math.abs(omegarps) < 1.0) {
            m_robotContainer.drivetrain.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds,
                VecBuilder.fill(.9, .9, 999999));
          }
        }
      }

    });
  }

  @Override
  public void disabledInit() {
    m_robotContainer.drivetrain.configNeutralMode(NeutralModeValue.Coast);
    m_autonomousCommand = null;
  }

  @Override
  public void disabledPeriodic() {

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand instanceof AutoCommand) {
      if (m_robotContainer.driverController.getHID().getBackButtonPressed()) {
        AutoCommand autoCmd = (AutoCommand) m_autonomousCommand;
        m_robotContainer.drivetrain.resetPose(autoCmd.getProperFlippedStartingPose());
      }
    }

  }

  @Override
  public void disabledExit() {

    m_robotContainer.drivetrain.configNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void autonomousInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void autonomousExit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.drivetrain.setControl(m_robotContainer.driveRC.withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(0));

  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {

  }
}
