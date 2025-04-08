package frc.robot.commands.auto;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ReefSide;
import frc.robot.RobotContainer;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.input.SnapButton;
import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem.RequestState;

public class Testing extends AutoCommand {
  private final PathPlannerPath path1;

  public Testing(ElevClArmSubsystem e, CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric swerve) {
    path1 = AutoBuildingBlocks.loadPathOrThrow("testing.1");
    addCommands(
        // Commands.waitSeconds(0.01),
        AutoBuildingBlocks.autoStep("PATH 1"),
        new PathAndElevateWithinDist(path1, ReefSide.FC, SnapButton.Right, 1.5, RequestState.CoralLevel4, e),
        AutoBuildingBlocks.autoStep("SHOOT"),
        // new LimelightAutoCmd(robot, ReefSide.FC, e, drivetrain, SnapButton.Right, RequestState.CoralLevel4, swerve, 2),
        AutoBuildingBlocks.autoStep("DONE")
    );
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
