package frc.robot.commands.auto;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ReefSide;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.input.SnapButton;
import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem.RequestState;

public class Left3Piece extends AutoCommand {
  private final PathPlannerPath path1;
  private final PathPlannerPath path2;
  private final PathPlannerPath path3;
  private final PathPlannerPath path4;
  private final PathPlannerPath path5;
  private final PathPlannerPath path6;

  public Left3Piece(ElevClArmSubsystem e, CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric swerve) {
    path1 = AutoBuildingBlocks.loadPathOrThrow("Left.1");
    path2 = AutoBuildingBlocks.loadPathOrThrow("Left.2");
    path3 = AutoBuildingBlocks.loadPathOrThrow("Left.3");
    path4 = AutoBuildingBlocks.loadPathOrThrow("Left.4");
    path5 = AutoBuildingBlocks.loadPathOrThrow("Left.5");
    path6 = AutoBuildingBlocks.loadPathOrThrow("Left.6");
    addCommands(
        // is auto step the problem??
        new WaitCommand(0.1),
        AutoBuildingBlocks.autoStep("PATH 1"),
        new PathAndElevateWithinDist(path1, ReefSide.BL, SnapButton.Right, 1.5, RequestState.CoralLevel4, e),
        AutoBuildingBlocks.autoStep("SCORE L4 RIGHT BL"),
        new LimelightAutoCmd(ReefSide.BL, e, drivetrain, SnapButton.Right, RequestState.CoralLevel4, swerve, 2),
        AutoBuildingBlocks.autoStep("PATH 2"),
        AutoBuildingBlocks.followPathCommand(path2),
        AutoBuildingBlocks.autoStep("PATH 3"),
        new PathAndElevateWithinDist(path3, ReefSide.FL, SnapButton.Left, 1.5, RequestState.CoralLevel4, e),
        AutoBuildingBlocks.autoStep("SCORE L4 LEFT FL"),
        new LimelightAutoCmd(ReefSide.FL, e, drivetrain, SnapButton.Left, RequestState.CoralLevel4, swerve, 2),
        AutoBuildingBlocks.autoStep("PATH 4"),
        AutoBuildingBlocks.followPathCommand(path4),
        AutoBuildingBlocks.autoStep("PATH 5"),
        new PathAndElevateWithinDist(path5, ReefSide.FL, SnapButton.Right, 1.5, RequestState.CoralLevel4, e),
        AutoBuildingBlocks.autoStep("3 PIECEEE BABY"),
        new LimelightAutoCmd(ReefSide.FL, e, drivetrain, SnapButton.Right, RequestState.CoralLevel4, swerve, 2),
        AutoBuildingBlocks.autoStep("PATH 6"),
        AutoBuildingBlocks.followPathCommand(path6),
        AutoBuildingBlocks.autoStep("DONE")
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
