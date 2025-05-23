// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
  final Field2d m_field2 = new Field2d();
  final Field2d m_field3 = new Field2d();

  double lastDashboardUpdate = 0;

  Timer climbToCoast;

  public static Alliance alliance = Alliance.Blue; // Default
  // Controls all configs for comp/practice bot
  public static final boolean isCompBot = true;

  private Timer warmupCommandTimer = new Timer();
  private Command warmupCommandNotAtStartPose = null;
  private Command warmupCommandAtStartPose = null;

  public Robot() {
    m_robotContainer = new RobotContainer();
   DataLogManager.start();
   DriverStation.startDataLog(DataLogManager.getLog());

    updateAlliance();
    m_robotContainer.updateDrivetrainRobotPerspective();

    climbToCoast = new Timer();
  }

  public static void updateAlliance() {
    alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  @Override
  public void robotInit() {
    warmupCommandNotAtStartPose = new WarmupAutoCmd(m_robotContainer.drivetrain, m_robotContainer.driveRC, true)
        .ignoringDisable(true);
    warmupCommandNotAtStartPose.schedule();
    warmupCommandTimer.reset();
    warmupCommandTimer.start();
    SmartDashboard.putBoolean("warmupNotAtStartPose finished", false);
    SmartDashboard.putBoolean("warmupAtStartPose finished", false);
    m_robotContainer.autoChooser.onChange(this::updateFieldPaths);
    updateAlliance();
    m_robotContainer.drivetrain.configNeutralMode(NeutralModeValue.Brake);   

    TimingUtils.logDuration("commandSwerveDriveTrain.addVisionMeasurement2", () -> {
    });
    TimingUtils.logDuration("ClimberSubsystem.periodic", () -> {
    });
    TimingUtils.logDuration("CliberSubsystem.printDashboard", () -> {
    });
    TimingUtils.logDuration("ElevClArmSubsystem.periodic", () -> {
    });
    TimingUtils.logDuration("ElevClArmSubsystem.update_coral_dio", () -> {
    });
    TimingUtils.logDuration("ElevClArmSubsystem.first_switch", () -> {
    });
    TimingUtils.logDuration("ElevClArmSubsystem.second_switch", () -> {
    });
    TimingUtils.logDuration("ElevClArmSubsystem.go", () -> {
    });
    TimingUtils.logDuration("ElevClArmSubsystem.atPosition", () -> {
    });
    TimingUtils.logDuration("ElevClArmSubsystem.atFinalPosition", () -> {
    });
    TimingUtils.logDuration("ElevClArmSubsystem.getEMode", () -> {
    });
    TimingUtils.logDuration("ElevClArmSubsystem.printDashboard", () -> {
    });
    TimingUtils.logDuration("PigeonSubsystem.periodic", () -> {
    });
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
    SmartDashboard.putString("snap", m_robotContainer.snap.toString());
    TimingUtils.logDuration("robotPeriodic", () -> {
      TimingUtils.logDuration("commandScheduler", () -> {
        CommandScheduler.getInstance().run();
      });

      TimingUtils.logDuration("dashboard", () -> {
        if (Timer.getFPGATimestamp() > lastDashboardUpdate + 0.200) {
          SmartDashboard.putData("Field", m_field);
          m_robotContainer.e.printDashboard();
          m_robotContainer.climber.printDashboard();
          lastDashboardUpdate = Timer.getFPGATimestamp();
          var driveState = m_robotContainer.drivetrain.getState();
          m_field.setRobotPose(driveState.Pose);
          SmartDashboard.putBoolean("RobotThinksItIsOnRed", alliance == Alliance.Red);
          SmartDashboard.putNumber("idlooking", m_robotContainer.idToLookFor);

          // if (m_robotContainer.driverController.getHID().getBackButtonPressed()) {
          // updateAlliance();
          // updateFieldPaths(m_robotContainer.autoChooser.getSelected());
          // }
        }
      });

      TimingUtils.logDuration("limelight", () -> {
        var botState = m_robotContainer.drivetrain.getState();
        double omegarps = Units.radiansToRotations(botState.Speeds.omegaRadiansPerSecond);

        for (int i = 0; i < limelights.length; ++i) {
          LimelightHelpers.SetRobotOrientation(limelights[i], botState.Pose.getRotation().getDegrees(), 0, 0, 0, 0,
              0);
          PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelights[i]);
          if (DriverStation.isEnabled()) {
            if (poseEstimate != null && poseEstimate.tagCount > 0 && Math.abs(omegarps) < 1.0) {
              m_robotContainer.drivetrain.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds,
                  VecBuilder.fill(.9, .9, 999999));
            }
          }
        }
      });

      // Run warmup commands.
      // Once the "not at the start pose" command is done, start the "at start pose"
      // warmup command
      if (warmupCommandNotAtStartPose != null) {
        if (warmupCommandTimer.hasElapsed(6)) {
          if (warmupCommandAtStartPose == null) {
            SmartDashboard.putBoolean("warmupNotAtStartPose finished", true);
            warmupCommandAtStartPose = new WarmupAutoCmd(m_robotContainer.drivetrain, m_robotContainer.driveRC, true)
                .ignoringDisable(true);
            warmupCommandAtStartPose.schedule();
            warmupCommandTimer.reset();
          } else if (warmupCommandTimer.hasElapsed(6)) {
            SmartDashboard.putBoolean("warmupAtStartPose finished", true);
            warmupCommandNotAtStartPose = null; // Prevent entering this block again; we have updated dashboard.
          }
        }
      }

    });
  }

  @Override
  public void disabledInit() {
  //  m_robotContainer.drivetrain.configNeutralMode(NeutralModeValue.Coast);
    m_autonomousCommand = null; // Reset autonomous command when disabled
  }

  @Override
  public void disabledPeriodic() {
    // TimingUtils.logDuration("disabledPeriodic", () -> {
    if (climbToCoast.get() > 6 && climbToCoast.get() < 7) {
      //m_robotContainer.climber.climbMotor.setIdleCoastMode(); // drop robot after 6 seconds post match
      //m_robotContainer.e.shoulderMotor.setIdleCoastMode();
      //m_robotContainer.e.rightElevatorMotor.setIdleCoastMode();
    }

    // Get autonomous command while disabled if not already set
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand instanceof AutoCommand) {
      if (m_robotContainer.driverController.getHID().getBackButtonPressed()) {
        AutoCommand autoCmd = (AutoCommand) m_autonomousCommand;
        m_robotContainer.drivetrain.resetPose(autoCmd.getProperFlippedStartingPose());

        updateAlliance();
        updateFieldPaths(m_robotContainer.autoChooser.getSelected());
      }
    }

    // });
  }

  @Override
  public void disabledExit() {
  //  m_robotContainer.climber.climbMotor.setIdleBrakeMode();
  //  m_robotContainer.drivetrain.configNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void autonomousInit() {
    // Reset pose based on the selected autonomous command
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

//    m_robotContainer.e.shoulderMotor.setIdleBrakeMode();
//    m_robotContainer.e.rightElevatorMotor.setIdleBrakeMode();
  }

  @Override
  public void teleopInit() {
  //  m_robotContainer.e.shoulderMotor.setIdleCoastMode();
  //  m_robotContainer.e.rightElevatorMotor.setIdleCoastMode();

    m_robotContainer.drivetrain.setControl(m_robotContainer.driveRC.withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(0));
  }

  @Override
  public void teleopPeriodic() {
    // TimingUtils.logDuration("teleopPeriodic", () -> {

    // });
  }

  @Override
  public void teleopExit() {
    climbToCoast.restart();
  }

  Orchestra m_orchestra;

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    // m_robotContainer.testController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    // m_robotContainer.testController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
    // m_robotContainer.testController.y().whileTrue(m_robotContainer.drivetrain.sysIdQuasistatic(Direction.kForward));
    // m_robotContainer.testController.b().whileTrue(m_robotContainer.drivetrain.sysIdQuasistatic(Direction.kReverse));
    // m_robotContainer.testController.a().whileTrue(m_robotContainer.drivetrain.sysIdDynamic(Direction.kForward));
    // m_robotContainer.testController.x().whileTrue(m_robotContainer.drivetrain.sysIdDynamic(Direction.kReverse));

    m_orchestra = new Orchestra();
    m_orchestra.addInstrument(m_robotContainer.drivetrain.getModule(0).getSteerMotor());
    m_orchestra.addInstrument(m_robotContainer.drivetrain.getModule(1).getSteerMotor());
    m_orchestra.addInstrument(m_robotContainer.drivetrain.getModule(2).getSteerMotor());
    m_orchestra.addInstrument(m_robotContainer.drivetrain.getModule(3).getSteerMotor());
    m_orchestra.addInstrument(m_robotContainer.drivetrain.getModule(0).getDriveMotor());
    m_orchestra.addInstrument(m_robotContainer.drivetrain.getModule(1).getDriveMotor());
    m_orchestra.addInstrument(m_robotContainer.drivetrain.getModule(2).getDriveMotor());
    m_orchestra.addInstrument(m_robotContainer.drivetrain.getModule(3).getDriveMotor());
    m_orchestra.addInstrument(m_robotContainer.climber.climbMotor.motor);
    m_orchestra.addInstrument(m_robotContainer.e.clawMotor.motor);
    m_orchestra.addInstrument(m_robotContainer.e.rightElevatorMotor.motor);
    m_orchestra.addInstrument(m_robotContainer.e.leftElevatorMotor.motor);
    m_orchestra.addInstrument(m_robotContainer.e.shoulderMotor.motor);
    var status = m_orchestra.loadMusic("kendrick.chrp");
    if (!status.isOK()) {
      m_orchestra.play();
    }
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
    m_orchestra.close();
  }
}
