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
import frc.robot.RobotContainer.ControlMode;
import frc.robot.commands.ClimberCmd;
import frc.robot.commands.ShootCmd;
import frc.robot.input.AnalogTrigger;
import frc.robot.input.DPadButton;
import frc.robot.input.Keybind;
import frc.robot.input.AnalogTrigger.Axis;
import frc.robot.input.DPadButton.DPad;
import frc.robot.input.Keybind.Button;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PigeonSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimbState;
import frc.robot.subsystems.ElevClArmSubsystem.ClawState;

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
  
  //modes keybind
  Keybind selectButton;
  Keybind startButton;
  
  //shoot keybind
  AnalogTrigger rightTrigger;
  AnalogTrigger leftTrigger;
  
  
  public enum ControlMode {
    Coral, Algae, Climb;
  }

  public ControlMode mode = ControlMode.Coral;

  public RobotContainer() {
    gyro = new PigeonSubsystem();
    pdp = new PowerDistribution(Constants.PDP_ID, ModuleType.kCTRE);
    
    limelightLeft = new LimelightSubsystem("limelight-left");
    limelightRight = new LimelightSubsystem("limelight-right");

    e = new ElevClArmSubsystem();
    climber = new ClimberSubsystem();
    led = new LEDSubsystem(this);
    
    // digitalio = new DigitalIOSubsystem(arm, shooter, floorIntake, climber); // if adam wants buttons again

      
      configureBindings();
    }
    
  private void configureBindings() {
    // climber engage - toggle? change controls when climbed? make sure arm is forward, and everything is safe
    selectButton = new Keybind(driverController, Button.Select);
    startButton = new Keybind(driverController, Button.Start);
    
    // processor (just shoot in safe?) maybe default to processor rather than algaesafe
    rightTrigger = new AnalogTrigger(driverController, Axis.RT, 0.5);

    selectButton.trigger().and(startButton).onTrue(new InstantCommand(() -> mode = ControlMode.Climb));
    startButton.trigger().and(selectButton.trigger().negate()).onTrue(new InstantCommand(() -> mode = ControlMode.Coral));
    selectButton.trigger().and(startButton.trigger().negate()).onTrue(new InstantCommand(() -> mode = ControlMode.Algae));



    rightTrigger.trigger().and(() -> mode == ControlMode.Coral).onTrue(new ShootCmd(e, ClawState.Poop));
    rightTrigger.trigger().and(() -> mode == ControlMode.Algae).onTrue(new ShootCmd(e, ClawState.Vomit));
    rightTrigger.trigger().and(() -> mode == ControlMode.Climb).onTrue(new ClimberCmd(climber, ClimbState.Down));
    
    leftTrigger.trigger().and(() -> mode == ControlMode.Climb).onTrue(new ClimberCmd(climber, ClimbState.Up));

    // go to lvl 1
    // go to lvl 2
    // go to lvl 3
    // go to lvl 4
      // ElevClArm, led, enum declaring the different levels
    

    // algae mode activate
      // ElevClArm, Led, toggle, purple leds

    // algae position bottom
    // algae position top
    //ElevClArm, check if algaemode, prob enum level
    
    // barge
    // elevclarm, enum position
    
    
      // snap to reef left
      // snap to reef right
      //Drive, Limelight
      
      // drive
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
