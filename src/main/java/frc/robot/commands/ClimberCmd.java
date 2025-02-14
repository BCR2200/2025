package frc.robot.commands;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimbState;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberCmd extends Command {
  private final ClimberSubsystem climber;
  public ClimbState state;

  public ClimberCmd(ClimberSubsystem climber, ClimbState state) {
    this.climber = climber;
    this.state = state;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.climbState = state;
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    climber.climbState = ClimbState.Off;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}