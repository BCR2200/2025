// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.AutoCommand;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  final Field2d m_field = new Field2d();
  final Field2d m_field2 = new Field2d();
  final Field2d m_field3 = new Field2d();

  double lastDashboardUpdate= 0;
  
  Timer climbToCoast;

  public static Alliance alliance = Alliance.Blue; // Default
  // Controls all configs for comp/practice bot
  public static final boolean isCompBot = true;

  public Robot() {
    PathfindingCommand.warmupCommand().schedule();

    m_robotContainer = new RobotContainer();
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    updateAlliance();
    if (alliance == Alliance.Red) {
      m_robotContainer.drivetrain.setOperatorPerspectiveForward(new Rotation2d(Math.PI));
    } else {
      m_robotContainer.drivetrain.setOperatorPerspectiveForward(new Rotation2d(0));
    }

    climbToCoast = new Timer();
  }

  public static void updateAlliance() {
    alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
  }

  @Override
  public void robotInit() {
    m_robotContainer.autoChooser.onChange(this::updateFieldPaths);
    updateAlliance();
  }

  public void updateFieldPaths(AutoCommand auto){
    if (auto != null){
      m_field.getObject("path").setPoses(auto.getAllProperFlippedPathPoses());
    } else {
      m_field.getObject("path").setPoses();
    }
  }

  @Override
  public void robotPeriodic() {
    // TimingUtils.logDuration("robotPeriodic", () -> {
      CommandScheduler.getInstance().run();
      if(Timer.getFPGATimestamp()> lastDashboardUpdate +0.200){
        SmartDashboard.putData("Field", m_field);
        m_robotContainer.e.printDashboard();
        m_robotContainer.climber.printDashboard();
        lastDashboardUpdate = Timer.getFPGATimestamp();
        var driveState = m_robotContainer.drivetrain.getState();
        m_field.setRobotPose(driveState.Pose);
        SmartDashboard.putBoolean("RobotThinksItIsOnRed", alliance == Alliance.Red);
        SmartDashboard.putNumber("idlooking", m_robotContainer.idToLookFor);

        if (m_robotContainer.driverController.getHID().getBackButtonPressed()) {
          updateAlliance();
          updateFieldPaths(m_robotContainer.autoChooser.getSelected());
        }
      }
    // });
  }

  @Override
  public void disabledInit() {
    m_robotContainer.drivetrain.configNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void disabledPeriodic() {
    // TimingUtils.logDuration("disabledPeriodic", () -> {
      if (climbToCoast.get() > 6 && climbToCoast.get() < 7) {
        m_robotContainer.climber.climbMotor.setIdleCoastMode(); // drop robot after 6 seconds post match
      }
    // });
  }

  @Override
  public void disabledExit() {
    m_robotContainer.climber.climbMotor.setIdleBrakeMode();
    m_robotContainer.drivetrain.configNeutralMode(NeutralModeValue.Brake);

  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    Elastic.selectTab("Auto");

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

    m_robotContainer.e.shoulderMotor.setIdleBrakeMode();
  }

  @Override
  public void teleopInit() {
    m_robotContainer.e.shoulderMotor.setIdleCoastMode();
    Elastic.selectTab("Teleoperated");
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
