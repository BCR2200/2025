package frc.robot.commands;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimbState;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberPositionCmd extends Command {
  private final ClimberSubsystem climber;
  public ClimbState state;

  public ClimberPositionCmd(ClimberSubsystem climber, ClimbState state) {
    this.climber = climber;
    this.state = state;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.setHeight(state);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return climber.atPosition();
  }
}