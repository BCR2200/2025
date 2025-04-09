package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimbHeight;
import frc.robot.subsystems.ClimberSubsystem.ManualClimbState;

public class ClimberCmd extends Command {
  private final ClimberSubsystem climber;
  private final ManualClimbState manualState;
  private final ClimbHeight targetHeight;

  /**
   * Creates a new ClimberCmd for manual control.
   * 
   * @param climber The climber subsystem
   * @param state The manual climb state to set
   */
  public ClimberCmd(ClimberSubsystem climber, ManualClimbState state) {
    this(climber, state, null);
  }

  /**
   * Creates a new ClimberCmd for setting a climb height.
   * 
   * @param climber The climber subsystem
   * @param height The target climb height to set
   */
  public ClimberCmd(ClimberSubsystem climber, ClimbHeight height) {
    this(climber, null, height);
  }

  private ClimberCmd(ClimberSubsystem climber, ManualClimbState state, ClimbHeight height) {
    this.climber = climber;
    this.manualState = state;
    this.targetHeight = height;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    if (manualState != null) {
      climber.climbState = manualState;
    }
    if (targetHeight != null) {
      climber.setHeight(targetHeight);
    }
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    if (manualState != null) {
      climber.climbState = ManualClimbState.Off;
    }
  }

  @Override
  public boolean isFinished() {
    return false; 
  }
}