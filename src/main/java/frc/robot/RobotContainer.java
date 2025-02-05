// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ClimberCmd;
import frc.robot.commands.RequesteStateCmd;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.RequesteStateCmd;
import frc.robot.input.AnalogTrigger;
import frc.robot.input.Keybind;
import frc.robot.input.AnalogTrigger.Axis;
import frc.robot.input.Keybind.Button;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PigeonSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimbState;
import frc.robot.subsystems.ElevClArmSubsystem.ClawState;
import frc.robot.subsystems.ElevClArmSubsystem.ControlMode;
import frc.robot.subsystems.ElevClArmSubsystem.RequestState;

public class RobotContainer {
  public final CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);

  // set up Subsystems
  public PigeonSubsystem gyro;
  public LimelightSubsystem limelightLeft;
  public LimelightSubsystem limelightRight;
  public ClimberSubsystem climber;
  public ElevClArmSubsystem e;
  public LEDSubsystem led;
  public PowerDistribution pdp;

  // modes keybind
  Keybind selectButton;
  Keybind startButton;
  
  // buttons
  Keybind aButton;
  Keybind bButton;
  Keybind xButton;
  Keybind yButton;

  // shoot keybind
  AnalogTrigger rightTrigger;
  AnalogTrigger leftTrigger;

  public RobotContainer() {
    gyro = new PigeonSubsystem();
    pdp = new PowerDistribution(Constants.PDP_ID, ModuleType.kCTRE);

    limelightLeft = new LimelightSubsystem("limelight-left");
    limelightRight = new LimelightSubsystem("limelight-right");

    e = new ElevClArmSubsystem();
    climber = new ClimberSubsystem();
    led = new LEDSubsystem(this);

    // digitalio = new DigitalIOSubsystem(arm, shooter, floorIntake, climber); // if
    // adam wants buttons again

    configureBindings();
  }

  private void configureBindings() {
    // climber engage - toggle? change controls when climbed? make sure arm is
    // forward, and everything is safe
    selectButton = new Keybind(driverController, Button.Select);
    startButton = new Keybind(driverController, Button.Start);
    aButton = new Keybind(driverController, Button.A);
    bButton = new Keybind(driverController, Button.B);
    xButton = new Keybind(driverController, Button.X);
    yButton = new Keybind(driverController, Button.Y);

    // processor (just shoot in safe?) maybe default to processor rather than
    // algaesafe
    rightTrigger = new AnalogTrigger(driverController, Axis.RT, 0.5);
    
    //select modes
    selectButton.trigger().and(startButton.trigger())
        .onTrue(new InstantCommand(() -> e.requestMode(ControlMode.Climb)));
    startButton.trigger().and(selectButton.trigger().negate())
        .onTrue(new InstantCommand(() -> e.requestMode(ControlMode.Coral)));
    selectButton.trigger().and(startButton.trigger().negate())
        .onTrue(new InstantCommand(() -> e.requestMode(ControlMode.Algae)));
    
    //shoot
    rightTrigger.trigger().and(() -> e.getEMode() == ControlMode.Coral)
        .whileTrue(new ShootCmd(e));
    rightTrigger.trigger().and(() -> e.getEMode() == ControlMode.Algae)
        .whileTrue(new ShootCmd(e));
    
    //climb up and down
    rightTrigger.trigger().and(() -> e.getEMode() == ControlMode.Climb)
        .whileTrue(new ClimberCmd(climber, ClimbState.Down));
    leftTrigger.trigger().and(() -> e.getEMode() == ControlMode.Climb)
        .whileTrue(new ClimberCmd(climber, ClimbState.Up));

    // request states for elevclarm
    // include negative feedback (rumble) for unavailable changes of state/mode

    // go to lvl 1
    aButton.trigger().and(() -> e.getEMode() == ControlMode.Coral)
        .whileTrue(new RequesteStateCmd(e, RequestState.CoralLevel1));
    //go to lvl 2
    bButton.trigger().and(() -> e.getEMode() == ControlMode.Coral)
        .whileTrue(new RequesteStateCmd(e, RequestState.CoralLevel2));
    //go to lvl 3
    xButton.trigger().and(() -> e.getEMode() == ControlMode.Coral)
        .whileTrue(new RequesteStateCmd(e, RequestState.CoralLevel3));
    //go to lvl 4    
    yButton.trigger().and(() -> e.getEMode() == ControlMode.Coral)
        .whileTrue(new RequesteStateCmd(e, RequestState.CoralLevel4));

    // algae position bottom
    aButton.trigger().and(() -> e.getEMode() == ControlMode.Algae)
        .whileTrue(new RequesteStateCmd(e, RequestState.AlgaeBottom));
    //algae position top
    bButton.trigger().and(() -> e.getEMode() == ControlMode.Algae)
        .whileTrue(new RequesteStateCmd(e, RequestState.AlgaeTop));
    // barge
    yButton.trigger().and(() -> e.getEMode() == ControlMode.Algae)
      .whileTrue(new RequesteStateCmd(e, RequestState.Barge));
    //processor
    xButton.trigger().and(() -> e.getEMode() == ControlMode.Algae)
      .whileTrue(new RequesteStateCmd(e, RequestState.Processor));

    //unjam
    leftTrigger.trigger().and(() -> e.getEMode() == ControlMode.Coral)
      .whileTrue(new RequesteStateCmd(e, RequestState.UnjamStrat1));
      
    // reg drive
    // snap to reef left
    // snap to reef right
      // Drive, Limelight
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
