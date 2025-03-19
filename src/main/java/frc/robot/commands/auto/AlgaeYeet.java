package frc.robot.commands.auto;

import frc.robot.ExtraMath;
import frc.robot.LimelightHelpers;
import frc.robot.OURLimelightHelpers;
import frc.robot.ReefSide;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.input.SnapButton;
import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem.RequestState;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeYeet extends Command {
  private final ElevClArmSubsystem e;

  public final RequestState state;
  private final double ep;
  Timer shootTimer;
  Timer abandonTimer;

  // doesn't shoot, end driving away

  private boolean finished = false;
  
    public AlgaeYeet(ElevClArmSubsystem e, RequestState state) {
      this(e, state, 0.25);
    }
  
    public AlgaeYeet(ElevClArmSubsystem e, RequestState state, double epsilon) {
      this.e = e;
      this.state = state;
      this.ep = epsilon;
      shootTimer = new Timer();


      abandonTimer = new Timer();
  
      addRequirements(e); // TODO should this be required? We do command the drivetrain...
    }
  
    @Override
    public void initialize() {
      shootTimer.reset();
      finished = false;

      e.requestState(state);
  }

  @Override
  public void execute() {

    // e movement stuff
    if (e.atFinalPosition(ep) && shootTimer.get() == 0) {
      e.shootLust = true;
      shootTimer.restart();
    }
    if (shootTimer.get() > 0.3) {
      e.shootLust = false;
      e.requestState(RequestState.None);
    }

    if (shootTimer.get() > 0.4) {
      finished = true; // blame Adam for this brain death
    }
  }

  @Override
  public void end(boolean interrupted) {
    // still stow if interrupted
    e.requestState(RequestState.None);
    e.shootLust = false;
  }

  @Override
  public boolean isFinished() {
    return finished;
  }

}
