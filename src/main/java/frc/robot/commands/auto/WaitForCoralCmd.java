package frc.robot.commands.auto;

import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem.RequestState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class WaitForCoralCmd extends Command {
  private final ElevClArmSubsystem e;
  private boolean finished;

  public WaitForCoralCmd(ElevClArmSubsystem e) {
    this.e = e;
  }

  @Override
  public void initialize() {
    finished = false;
  }

  @Override
  public void execute() {
    if (e.isCoralInHopper() || e.isCoralEnteredClaw() || e.isCoralLeavingClaw()){
      finished = true;
    }
  }
  
  @Override
  public boolean isFinished() {
    return finished;
  }

  @Override
  public void end(boolean interrupted) {
  }

}
