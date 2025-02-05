package frc.robot.commands;

import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem.RequestState;
import edu.wpi.first.wpilibj2.command.Command;

public class RequesteStateCmd extends Command {
  private final ElevClArmSubsystem e;
  private final RequestState state;

  public RequesteStateCmd(ElevClArmSubsystem e, RequestState state) {
    this.e = e;
    this.state=state;
    addRequirements(e);
  }

  @Override
  public void initialize() {
    e.requestState = state; 
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    e.requestState = RequestState.None;
  }

}
