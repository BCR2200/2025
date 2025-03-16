package frc.robot.commands.auto;

import frc.robot.ReefSide;
import frc.robot.input.SnapButton;
import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem.RequestState;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PathAndElevateWithinDist extends SequentialCommandGroup {

  public PathAndElevateWithinDist(PathPlannerPath path, ReefSide side, 
      SnapButton snap, double distance, RequestState state, ElevClArmSubsystem e) {
    addCommands(
      new ParallelCommandGroup(
          AutoBuildingBlocks.followPathCommand(path),
          new SequentialCommandGroup(
            new LimelightDeadline(side, snap, distance),
            new AutoRequesteStateCmd(e, state)
          )
        )
    );
  }
}
