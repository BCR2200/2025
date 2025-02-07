// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ClimberSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
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
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    // if (m_robotContainer.driverController.getHID().getPOV() == 0) {
    //   m_robotContainer.e.leftElevatorMotor.setPercentOutput(0.1);
    // } else if (m_robotContainer.driverController.getHID().getPOV() == 180) {
    //   m_robotContainer.e.leftElevatorMotor.setPercentOutput(-0.1);
    // } else {
    //   m_robotContainer.e.leftElevatorMotor.setPercentOutput(0);
    // }

    if (m_robotContainer.driverController.getHID().getPOV() == 270) {
      m_robotContainer.e.rightElevatorMotor.setPercentOutput(-0.1);
    } else if (m_robotContainer.driverController.getHID().getPOV() == 90) {
      m_robotContainer.e.rightElevatorMotor.setPercentOutput(0.1);
    } else{
      m_robotContainer.e.rightElevatorMotor.setPercentOutput(0);
      
    }

    if (m_robotContainer.driverController.getHID().getXButton()) {
      m_robotContainer.e.shoulderMotor.setPercentOutput(0.1);
    } else if (m_robotContainer.driverController.getHID().getAButton()) {
      m_robotContainer.e.shoulderMotor.setPercentOutput(-0.1);
    } else{
      m_robotContainer.e.shoulderMotor.setPercentOutput(0);

    }

    if (m_robotContainer.driverController.getHID().getYButton()) {
      m_robotContainer.climber.climbMotor.setPercentOutput(0.1);
    } else if (m_robotContainer.driverController.getHID().getBButton()) {
      m_robotContainer.climber.climbMotor.setPercentOutput(-0.1);
    } else{
      m_robotContainer.climber.climbMotor.setPercentOutput(0);

    }

    if (m_robotContainer.driverController.getHID().getRightTriggerAxis() > 0.1) {
      m_robotContainer.e.clawMotor.setPercentOutput(0.1);
    } else if (m_robotContainer.driverController.getHID().getLeftTriggerAxis() > 0.1) {
      m_robotContainer.e.clawMotor.setPercentOutput(-0.1);
    } else{
      m_robotContainer.e.clawMotor.setPercentOutput(0);

    }
  }

  @Override
  public void testExit() {
  }
}
