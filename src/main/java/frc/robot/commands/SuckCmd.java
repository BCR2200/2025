package frc.robot.commands;

import frc.robot.subsystems.ElevClArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class SuckCmd extends Command {
  private final ElevClArmSubsystem e;

  public SuckCmd(ElevClArmSubsystem e) {
    this.e = e;
    // do not require, sets position to none lol
  }

  @Override
  public void initialize() {
    e.suck = true;
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    e.suck = false;
  }

}
