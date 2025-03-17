package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.drive.CommandSwerveDrivetrain;

public class WarmupAutoCmd extends AutoCommand {
  private final PathPlannerPath path1;

  public WarmupAutoCmd(CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric swerve, boolean updatePoseFirst) {
    path1 = AutoBuildingBlocks.loadPathOrThrow("testing.1");
    ArrayList<Command> commands = new ArrayList<>();
    if (updatePoseFirst) {
      commands.add(AutoBuildingBlocks.autoStep("Update Pose"));
      commands.add(AutoBuildingBlocks.resetOdom(drivetrain, path1));
    }

    commands.add(AutoBuildingBlocks.autoStep("PATH 1"));
    commands.add(AutoBuildingBlocks.followPathCommand(path1));
    commands.add(AutoBuildingBlocks.autoStep("DONE"));
    addCommands(commands.toArray(new Command[0]));
  }

  @Override
  List<Pose2d> getAllRawPathPoses() {
    return Stream.of(path1.getPathPoses())
        .flatMap(Collection::stream)
        .collect(Collectors.toList());
  }

  @Override
  public Pose2d getRawStartingPose() {
    return path1.getStartingHolonomicPose().orElseThrow();
  }

}
