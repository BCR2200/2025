package frc.robot.commands.auto;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.ReefSide;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.input.SnapButton;
import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem.RequestState;

public class LeftAuto extends AutoCommand {
  private final PathPlannerPath path1;
  private final PathPlannerPath path2;
  private final PathPlannerPath path3;
  private final PathPlannerPath path4;
  private final PathPlannerPath path5;

  public LeftAuto(ElevClArmSubsystem e, CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric swerve) {
    path1 = AutoBuildingBlocks.loadPathOrThrow("Left.1");
    path2 = AutoBuildingBlocks.loadPathOrThrow("Left.2");
    path3 = AutoBuildingBlocks.loadPathOrThrow("Left.3");
    path4 = AutoBuildingBlocks.loadPathOrThrow("Left.4");
    path5 = AutoBuildingBlocks.loadPathOrThrow("Left.5");
    addCommands(
        AutoBuildingBlocks.resetOdom(drivetrain, path1),
        AutoBuildingBlocks.autoStep("PATH 1"),
        AutoBuildingBlocks.followPathCommand(path1),
        AutoBuildingBlocks.autoStep("SCORE L4 LEFT BL"),
        new LimelightAutoCmd(ReefSide.BL, e, drivetrain, SnapButton.Left, RequestState.CoralLevel4, swerve, 2),
        AutoBuildingBlocks.autoStep("PATH 2"),
        AutoBuildingBlocks.followPathCommand(path2),
        AutoBuildingBlocks.autoStep("PATH 3"),
        AutoBuildingBlocks.followPathCommand(path3),
        AutoBuildingBlocks.autoStep("SCORE L4 LEFT FL"),
        new LimelightAutoCmd(ReefSide.FL, e, drivetrain, SnapButton.Left, RequestState.CoralLevel4, swerve, 2),
        AutoBuildingBlocks.autoStep("PATH 4"),
        AutoBuildingBlocks.followPathCommand(path4),
        AutoBuildingBlocks.autoStep("PATH 5"),
        AutoBuildingBlocks.followPathCommand(path5),
        AutoBuildingBlocks.autoStep("DONE")
    );
  }

  @Override
  List<Pose2d> getAllRawPathPoses() {
    return Stream.of(path1.getPathPoses(), path2.getPathPoses(), path3.getPathPoses(), path4.getPathPoses(), path5.getPathPoses())
        .flatMap(Collection::stream)
        .collect(Collectors.toList());
  }

  @Override
  public Pose2d getRawStartingPose() {
    return path1.getStartingHolonomicPose().orElseThrow();
  }

}
