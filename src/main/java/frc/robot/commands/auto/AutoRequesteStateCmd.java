package frc.robot.commands.auto;

import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem.RequestState;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoRequesteStateCmd extends Command {
  private final ElevClArmSubsystem e;
  private final RequestState state;
  private boolean finished;

  public AutoRequesteStateCmd(ElevClArmSubsystem e, RequestState state) {
    this.e = e;
    this.state=state;
    addRequirements(e);
  }

  @Override
  public void initialize() {
    e.requestState = state;
    finished = false;
  }

  @Override
  public void execute() {
    finished = true;
  }
  
  @Override
  public boolean isFinished() {
    return finished;
  }

  @Override
  public void end(boolean interrupted) {}

}
