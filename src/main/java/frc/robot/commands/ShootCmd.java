package frc.robot.commands;

import frc.robot.subsystems.ElevClArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootCmd extends Command {
  private final ElevClArmSubsystem e;

  public ShootCmd(ElevClArmSubsystem e) {
    this.e = e;
    // do not require, sets position to none lol
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    if(e.atPosition(5)==true){
      e.shootLust = true;
    }
    else{
      e.shootLust=false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    e.shootLust = false;
  }

}
