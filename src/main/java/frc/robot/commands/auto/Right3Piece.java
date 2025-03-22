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
import frc.robot.commands.RequesteStateCmd;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.input.SnapButton;
import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem.RequestState;

public class Right3Piece extends AutoCommand {
  private final PathPlannerPath path1;
  private final PathPlannerPath path2;
  private final PathPlannerPath path3;
  private final PathPlannerPath path4;
  private final PathPlannerPath path5;
  private final PathPlannerPath path6;

  public Right3Piece(ElevClArmSubsystem e, CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric swerve) {
    path1 = AutoBuildingBlocks.loadPathOrThrow("Right.1");
    path2 = AutoBuildingBlocks.loadPathOrThrow("Right.2");
    path3 = AutoBuildingBlocks.loadPathOrThrow("Right.3");
    path4 = AutoBuildingBlocks.loadPathOrThrow("Right.4");
    path5 = AutoBuildingBlocks.loadPathOrThrow("Right.5");
    path6 = AutoBuildingBlocks.loadPathOrThrow("Right.6");
    addCommands(
        // is auto step the problem??
        new WaitCommand(0.1),
        AutoBuildingBlocks.autoStep("PATH 1"),
        // new AutoRequesteStateCmd(e, RequestState.CoralLevel4),
        // AutoBuildingBlocks.followPathCommand(path1),
        new PathAndElevateWithinDist(path1, ReefSide.BR, SnapButton.Left, 1.5, RequestState.CoralLevel4, e),
        AutoBuildingBlocks.autoStep("SCORE L4 RIGHT BR"),
        new LimelightAutoCmd(ReefSide.BR, e, drivetrain, SnapButton.Left, RequestState.CoralLevel4, swerve, 2),
        AutoBuildingBlocks.autoStep("PATH 2"),
        AutoBuildingBlocks.followPathCommand(path2),
        AutoBuildingBlocks.autoStep("PATH 3"),
        new PathAndElevateWithinDist(path3, ReefSide.FR, SnapButton.Right, 1.5, RequestState.CoralLevel4, e),
        AutoBuildingBlocks.autoStep("SCORE L4 RIGHT FR"),
        Commands.race(
          new GiveUp(e),
          new LimelightAutoCmd(ReefSide.FR, e, drivetrain, SnapButton.Right, RequestState.CoralLevel4, swerve, 2)
        ),
        AutoBuildingBlocks.autoStep("PATH 4"),
        AutoBuildingBlocks.followPathCommand(path4),
        AutoBuildingBlocks.autoStep("PATH 5"),
        new PathAndElevateWithinDist(path5, ReefSide.FR, SnapButton.Left, 1.5, RequestState.CoralLevel4, e),
        AutoBuildingBlocks.autoStep("RAHHHHHH"),
        Commands.race(
          new GiveUp(e),
          new LimelightAutoCmd(ReefSide.FR, e, drivetrain, SnapButton.Left, RequestState.CoralLevel4, swerve, 2)
        ),
        AutoBuildingBlocks.autoStep("PATH 6"),
        AutoBuildingBlocks.followPathCommand(path6),
        AutoBuildingBlocks.autoStep("IS HE GOATED??"),
        // AutoBuildingBlocks.followPathCommand(path7),
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
