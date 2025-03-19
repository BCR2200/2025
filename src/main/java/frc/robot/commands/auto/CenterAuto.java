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

public class CenterAuto extends AutoCommand {
  private final PathPlannerPath path1;
  private final PathPlannerPath path2;
  private final PathPlannerPath path3;

  public CenterAuto(ElevClArmSubsystem e, CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric swerve) {
    path1 = AutoBuildingBlocks.loadPathOrThrow("Center.1");
    path2 = AutoBuildingBlocks.loadPathOrThrow("Center.2");
    path3 = AutoBuildingBlocks.loadPathOrThrow("Center.3");
    addCommands(
        AutoBuildingBlocks.autoStep("PATH 1"),
        AutoBuildingBlocks.followPathCommand(path1),
        AutoBuildingBlocks.autoStep("SCORE L4 CENTER BC"),
        new LimelightAutoCmd(ReefSide.BC, e, drivetrain, SnapButton.Right, RequestState.CoralLevel4, swerve, 2),
        AutoBuildingBlocks.autoStep("ALGAE TIME"),
        Commands.parallel(
          AutoBuildingBlocks.followPathCommand(path2),
          new AlgaeModeCmd(e)
          ),
          AutoBuildingBlocks.autoStep("GRAB ALGAE BC"),
          new AlgaeAutoCmd(ReefSide.BC, e, drivetrain, RequestState.AlgaeBottom, swerve, 2),
          AutoBuildingBlocks.autoStep("BARGE!!"),
          AutoBuildingBlocks.followPathCommand(path3),
          AutoBuildingBlocks.autoStep("YEEEEET!!"),
          new AlgaeYeet(e, RequestState.Barge, 2)
    );
  }

  @Override
  List<Pose2d> getAllRawPathPoses() {
    return Stream.of(path1.getPathPoses(),path2.getPathPoses(),path3.getPathPoses())
        .flatMap(Collection::stream)
        .collect(Collectors.toList());
  }

  @Override
  public Pose2d getRawStartingPose() {
    return path1.getStartingHolonomicPose().orElseThrow();
  }

}
