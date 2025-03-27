package frc.robot.commands.auto;

import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem.RequestState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class GiveUp extends Command {
  private final ElevClArmSubsystem e;
  private boolean finished;
  Timer giveupTimer;
  boolean coralDetected;

  public GiveUp(ElevClArmSubsystem e) {
    this.e = e;

    giveupTimer = new Timer();
  }

  @Override
  public void initialize() {
    finished = false;
    coralDetected = false;
    giveupTimer.start();
  }

  @Override
  public void execute() {
    if (e.isCoralInHopper() || e.isCoralInHopper() || e.isCoralLeavingClaw()){
      coralDetected = true;
    }
    if (giveupTimer.get() > 1 && !coralDetected){
      finished = true;
    } 
  }
  
  @Override
  public boolean isFinished() {
    return finished;
  }

  @Override
  public void end(boolean interrupted) {
    giveupTimer.stop();
    giveupTimer.reset();
  }

}
