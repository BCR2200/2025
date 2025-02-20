package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem.RequestState;

public class AutoStateShootCmd extends Command {
  private final ElevClArmSubsystem e;
  public RequestState state;
  boolean finished;
  Timer shootTimer;
  double ep;

  public AutoStateShootCmd(ElevClArmSubsystem e, RequestState state) {
    this(e, state, 0.25);
  }

  public AutoStateShootCmd(ElevClArmSubsystem e, RequestState state, double epsilon) {
    this.e = e;
    this.state = state;
    ep = epsilon;
    shootTimer = new Timer();
    addRequirements(e);
  }

  @Override
  public void initialize() {
    e.requestState(state);
    shootTimer.reset();
    finished = false;

  }

  @Override
  public void execute() {
    if (e.atPosition(ep) && state.finaleState() == e.state && shootTimer.get() == 0) {
       e.shootLust = true;
       shootTimer.start();
    }; 
    //TODO adjust the value
    if (shootTimer.get() > 0.3) {
      finished = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    e.requestState(RequestState.None);
    e.shootLust = false;
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}