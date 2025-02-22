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
import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.timing.TimingUtils;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  final Field2d m_field = new Field2d();
  final Field2d m_field2 = new Field2d();
  final Field2d m_field3 = new Field2d();

  double lastDashboardUpdate= 0;
  

  Timer climbToCoast;

  public Robot() {
    m_robotContainer = new RobotContainer();
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    // Do this in either robot or subsystem init

   // SmartDashboard.putData("Field Limelight", m_field2);
    //SmartDashboard.putData("Field Limelight-Right", m_field3);

    // LimelightHelpers.SetRobotOrientation("limelight-left", -90, 0.0, 0.0, 0.0, 0.0, 0.0);
    // why are we saying yaw = -90? shouldn't it be m_robotContainer.gyro.Y, below too?

    // if(llMeasurement.rawFiducials.length == 0){
    //   llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");
    // }


    Alliance alliance = DriverStation.getAlliance().orElse(null);
    if (alliance == Alliance.Red) {
      m_robotContainer.drivetrain.setOperatorPerspectiveForward(new Rotation2d(Math.PI));
    } else {
      m_robotContainer.drivetrain.setOperatorPerspectiveForward(new Rotation2d(0));
    }


    var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
    if (llMeasurement != null) {
      m_robotContainer.drivetrain.resetPose(llMeasurement.pose);
    }

    climbToCoast = new Timer();
  }

  @Override
  public void robotInit() {
   // PathfindingCommand.warmupCommand().schedule();
  }

  @Override
  public void robotPeriodic() {
    TimingUtils.logDuration("robotPeriodic", () -> {
      CommandScheduler.getInstance().run();
      if(Timer.getFPGATimestamp()> lastDashboardUpdate +0.500){
        SmartDashboard.putNumber("Wheel Velocity", m_robotContainer.drivetrain.getModule(0).getDriveMotor().getVelocity().getValueAsDouble());
        SmartDashboard.putData("Field", m_field);
        m_robotContainer.e.printDashboard();
        m_robotContainer.climber.printDashboard();
        lastDashboardUpdate = Timer.getFPGATimestamp();
        var driveState = m_robotContainer.drivetrain.getState();
        m_field.setRobotPose(driveState.Pose);
        }
      // SmartDashboard.putNumber(
      // "CAN Utilization %", RobotController.getCANStatus().percentBusUtilization * 100.0);
      // SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
      // SmartDashboard.putNumber("CPU Temperature", RobotController.getCPUTemp());
      // SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
      // SmartDashboard.putNumber("heightfactor", m_robotContainer.heightFactor);

      /*
       * This example of adding Limelight is very simple and may not be sufficient for
       * on-field use.
       * Users typically need to provide a standard deviation that scales with the
       * distance to target
       * and changes with number of tags available.
       *
       * This example is sufficient to show that vision integration is possible,
       * though exact implementation
       * of how to use vision should be tuned per-robot and to the team's
       * specification.
       */

    // double robotYaw = m_robotContainer.gyro.Y - 90; // TODO ???????
    // LimelightHelpers.SetRobotOrientation("limelight-left", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
    // LimelightHelpers.SetRobotOrientation("limelight-right", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

      // double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      // // Get left side Limelight measurment and add to Pose Estimator
      // LimelightHelpers.PoseEstimate limelightMeasurementLeft = LimelightHelpers
      //     .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
      // LimelightHelpers.PoseEstimate limelightMeasurementRight = LimelightHelpers
      //     .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");

      // if (limelightMeasurementLeft != null && limelightMeasurementLeft.tagCount > 0 && omegaRps < 2.0) {

      //   m_robotContainer.drivetrain.addVisionMeasurement(
      //       limelightMeasurementLeft.pose,
      //       Utils.fpgaToCurrentTime(limelightMeasurementLeft.timestampSeconds),
      //       VecBuilder.fill(0.05, 0.05, 9999)); // TODO add std dev here
      //       // within 5 cm in the x and y, ignore limelight rotation
      // }

      // if (limelightMeasurementRight != null && limelightMeasurementRight.tagCount > 0 && omegaRps < 2.0) {
      //   m_robotContainer.drivetrain.addVisionMeasurement(
      //       limelightMeasurementRight.pose,
      //       Utils.fpgaToCurrentTime(limelightMeasurementRight.timestampSeconds),
      //       VecBuilder.fill(0.05, 0.05, 9999)); // TODO add std dev here
      // }

      // print poses to dashboard
      // if (limelightMeasurementLeft != null) {
      //   m_field2.setRobotPose(limelightMeasurementLeft.pose);
      // }
      // if (limelightMeasurementRight != null) {
      //   m_field3.setRobotPose(limelightMeasurementRight.pose);
      // }

    });
  }

  @Override
  public void disabledInit() {
    m_robotContainer.drivetrain.configNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void disabledPeriodic() {
    TimingUtils.logDuration("disabledPeriodic", () -> {
      if (climbToCoast.get() > 6 && climbToCoast.get() < 7) {
        m_robotContainer.climber.climbMotor.setIdleCoastMode(); // drop robot after 6 seconds post match
      }
    });
  }

  @Override
  public void disabledExit() {
    m_robotContainer.climber.climbMotor.setIdleBrakeMode();
    m_robotContainer.drivetrain.configNeutralMode(NeutralModeValue.Brake);

  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    m_robotContainer.drivetrain.configNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void autonomousPeriodic() {
    TimingUtils.logDuration("autonomousPeriodic", () -> {

    });
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    TimingUtils.logDuration("teleopPeriodic", () -> {

    });
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
    TimingUtils.logDuration("testPeriodic", () -> {
      // if (m_robotContainer.driverController.getHID().getPOV() == 0) {
      // m_robotContainer.e.leftElevatorMotor.setPercentOutput(0.1);
      // } else if (m_robotContainer.driverController.getHID().getPOV() == 180) {
      // m_robotContainer.e.leftElevatorMotor.setPercentOutput(-0.1);
      // } else {
      // m_robotContainer.e.leftElevatorMotor.setPercentOutput(0);
      // }

      // if (m_robotContainer.driverController.getHID().getPOV() == 270) {
      // m_robotContainer.e.rightElevatorMotor.setPercentOutput(-0.1);
      // } else if (m_robotContainer.driverController.getHID().getPOV() == 90) {
      // m_robotContainer.e.rightElevatorMotor.setPercentOutput(0.1);
      // } else{
      // m_robotContainer.e.rightElevatorMotor.setPercentOutput(0);

      // }

      // if (m_robotContainer.driverController.getHID().getXButton()) {
      // m_robotContainer.e.shoulderMotor.setPercentOutput(0.1);
      // } else if (m_robotContainer.driverController.getHID().getAButton()) {
      // m_robotContainer.e.shoulderMotor.setPercentOutput(-0.1);
      // } else{
      // m_robotContainer.e.shoulderMotor.setPercentOutput(0);

      // }

      // if (m_robotContainer.driverController.getHID().getYButton()) {
      // m_robotContainer.climber.climbMotor.setPercentOutput(0.1);
      // } else if (m_robotContainer.driverController.getHID().getBButton()) {
      // m_robotContainer.climber.climbMotor.setPercentOutput(-0.1);
      // } else{
      // m_robotContainer.climber.climbMotor.setPercentOutput(0);

      // }

      // if (m_robotContainer.driverController.getHID().getRightTriggerAxis() > 0.1) {
      // m_robotContainer.e.clawMotor.setPercentOutput(0.1);
      // } else if (m_robotContainer.driverController.getHID().getLeftTriggerAxis() >
      // 0.1) {
      // m_robotContainer.e.clawMotor.setPercentOutput(-0.1);
      // } else{
      // m_robotContainer.e.clawMotor.setPercentOutput(0);

      // }

      // Run SysId routines when holding back/start and X/Y.
      // Note that each routine should be run exactly once in a single log.

    });
  }

  @Override
  public void testExit() {
    m_orchestra.close();
  }
}
