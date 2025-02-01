package frc.robot.commands;

import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem.ClawState;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootCmd extends Command {
  private final ElevClArmSubsystem e;
  public ClawState state;

  public ShootCmd(ElevClArmSubsystem e, ClawState state) {
    this.e = e;
    this.state = state;
    addRequirements(e);
  }

  @Override
  public void initialize() {
    e.shootLust = true;
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
  }

}
