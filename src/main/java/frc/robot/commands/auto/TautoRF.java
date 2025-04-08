package frc.robot.commands.auto;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ReefSide;
import frc.robot.RobotContainer;
import frc.robot.commands.RequesteStateCmd;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.input.SnapButton;
import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem.RequestState;

public class TautoRF extends AutoCommand {
  private final PathPlannerPath path1;
  private final PathPlannerPath path2;
  private final PathPlannerPath path3;

  public TautoRF(RobotContainer robot, ElevClArmSubsystem e, CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric swerve) {
    path1 = AutoBuildingBlocks.loadPathOrThrow("TautoRF.1");
    path2 = AutoBuildingBlocks.loadPathOrThrow("TautoRF.2");
    path3 = AutoBuildingBlocks.loadPathOrThrow("TautoRF.3");
    addCommands(
        // is auto step the problem??
        new WaitCommand(0.01),

        AutoBuildingBlocks.autoStep("PATH 1"),
        new PathAndElevateWithinDist(path1, ReefSide.FR, SnapButton.Left, 1.5, RequestState.CoralLevel3, e),
        AutoBuildingBlocks.autoStep("SCORE L3 Left FR"),
        new LimelightAutoCmd(robot, ReefSide.FR, e, drivetrain, SnapButton.Left, RequestState.CoralLevel3, swerve, 2),
        AutoBuildingBlocks.autoStep("Return to Feeder"),
        AutoBuildingBlocks.followPathCommand(path3),

        new WaitForCoralCmd(e),
        
        AutoBuildingBlocks.autoStep("PATH 1"),
        new PathAndElevateWithinDist(path1, ReefSide.FR, SnapButton.Right, 1.5, RequestState.CoralLevel3, e),
        AutoBuildingBlocks.autoStep("SCORE L3 RIGHT FR"),
        new LimelightAutoCmd(robot, ReefSide.FR, e, drivetrain, SnapButton.Right, RequestState.CoralLevel3, swerve, 2),
        AutoBuildingBlocks.autoStep("Return to Feeder"),
        AutoBuildingBlocks.followPathCommand(path2),

        new WaitForCoralCmd(e),
        
        AutoBuildingBlocks.autoStep("PATH 1"),
        new PathAndElevateWithinDist(path1, ReefSide.FR, SnapButton.Left, 1.5, RequestState.CoralLevel2, e),
        AutoBuildingBlocks.autoStep("SCORE L3 Left FR"),
        new LimelightAutoCmd(robot, ReefSide.FR, e, drivetrain, SnapButton.Left, RequestState.CoralLevel2, swerve, 2),
        AutoBuildingBlocks.autoStep("Return to Feeder"),
        AutoBuildingBlocks.followPathCommand(path3),

        new WaitForCoralCmd(e),
        
        AutoBuildingBlocks.autoStep("PATH 1"),
        new PathAndElevateWithinDist(path1, ReefSide.FR, SnapButton.Right, 1.5, RequestState.CoralLevel2, e),
        AutoBuildingBlocks.autoStep("SCORE L3 RIGHT FR"),
        new LimelightAutoCmd(robot, ReefSide.FR, e, drivetrain, SnapButton.Right, RequestState.CoralLevel2, swerve, 2),
        AutoBuildingBlocks.autoStep("Return to Feeder"),
        AutoBuildingBlocks.followPathCommand(path2)
    );
  }

  @Override
  List<Pose2d> getAllRawPathPoses() {
    return Stream.of(path1.getPathPoses(), path2.getPathPoses(), path3.getPathPoses())
        .flatMap(Collection::stream)
        .collect(Collectors.toList());
  }

  @Override
  public Pose2d getRawStartingPose() {
    return path1.getStartingHolonomicPose().orElseThrow();
  }

}
