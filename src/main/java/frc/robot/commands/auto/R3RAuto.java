package frc.robot.commands.auto;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ReefSide;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.input.SnapButton;
import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem.RequestState;

public class R3RAuto extends AutoCommand {
  private final PathPlannerPath path1;
  private final PathPlannerPath path2;
  private final PathPlannerPath path3;
  private final PathPlannerPath path4;
  private final PathPlannerPath path5;
  private final PathPlannerPath path6;

  public R3RAuto(ElevClArmSubsystem e, CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric swerve) {
    path1 = AutoBuildingBlocks.loadPathOrThrow("R3R.1");
    path2 = AutoBuildingBlocks.loadPathOrThrow("R3R.2");
    path3 = AutoBuildingBlocks.loadPathOrThrow("R3R.3");
    path4 = AutoBuildingBlocks.loadPathOrThrow("R3R.4");
    path5 = AutoBuildingBlocks.loadPathOrThrow("R3R.5");
    path6 = AutoBuildingBlocks.loadPathOrThrow("R3R.6");
    addCommands(
        AutoBuildingBlocks.resetOdom(drivetrain, path1),
        AutoBuildingBlocks.autoStep("PATH 1"),
        AutoBuildingBlocks.followPathCommand(path1),
        AutoBuildingBlocks.autoStep("SCORE L4 Right BR"),
        new LimelightAutoCmd(ReefSide.BR, e, drivetrain, SnapButton.Right, RequestState.CoralLevel4, swerve, 2),
        AutoBuildingBlocks.autoStep("PATH 2"),
        AutoBuildingBlocks.followPathCommand(path2),
        AutoBuildingBlocks.autoStep("PATH 3"),
        AutoBuildingBlocks.followPathCommand(path3),
        AutoBuildingBlocks.autoStep("SCORE L4 LEFT FL"),
        new LimelightAutoCmd(ReefSide.FR, e, drivetrain, SnapButton.Right, RequestState.CoralLevel4, swerve, 2),
        AutoBuildingBlocks.autoStep("PATH 4"),
        Commands.parallel(
          AutoBuildingBlocks.followPathCommand(path4),
          new AlgaeModeCmd(e)
          ),
          AutoBuildingBlocks.autoStep("GRAB ALGAE FR"),
          new AlgaeAutoCmd(ReefSide.FR, e, drivetrain, RequestState.AlgaeBottom, swerve, 2),
          AutoBuildingBlocks.autoStep("GUN IT"),
          AutoBuildingBlocks.followPathCommand(path5),
          AutoBuildingBlocks.autoStep("IS HE GOATED???"),
          AutoBuildingBlocks.followPathCommand(path6)
    );
  }

  @Override
  List<Pose2d> getAllRawPathPoses() {
    return Stream.of(path1.getPathPoses(), path2.getPathPoses(), path3.getPathPoses(), path4.getPathPoses(), path5.getPathPoses(), path6.getPathPoses())
        .flatMap(Collection::stream)
        .collect(Collectors.toList());
  }

  @Override
  public Pose2d getRawStartingPose() {
    return path1.getStartingHolonomicPose().orElseThrow();
  }

}
