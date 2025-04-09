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

public class TautoLB extends AutoCommand {
  private final PathPlannerPath path1L;
  private final PathPlannerPath path1R;
  private final PathPlannerPath path2L;
  private final PathPlannerPath path2R;

  public TautoLB(RobotContainer robot, ElevClArmSubsystem e, CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric swerve) {
    path1L = AutoBuildingBlocks.loadPathOrThrow("TautoLB.1L");
    path1R = AutoBuildingBlocks.loadPathOrThrow("TautoLB.1R");
    path2L = AutoBuildingBlocks.loadPathOrThrow("TautoLB.2L");
    path2R = AutoBuildingBlocks.loadPathOrThrow("TautoLB.2R");
    addCommands(
        // is auto step the problem??
        new WaitForCoralCmd(e),

        AutoBuildingBlocks.autoStep("PATH 1"),
        new PathAndElevateWithinDist(path1L, ReefSide.BL, SnapButton.Left, 1.5, RequestState.CoralLevel3, e),
        AutoBuildingBlocks.autoStep("SCORE L3 Left BL"),
        new LimelightAutoCmd(robot, ReefSide.BL, e, drivetrain, SnapButton.Left, RequestState.CoralLevel3, swerve, 2),
        AutoBuildingBlocks.autoStep("Return to Feeder"),
        AutoBuildingBlocks.followPathCommand(path2L),

        new WaitForCoralCmd(e),
        
        AutoBuildingBlocks.autoStep("PATH 1R"),
        new PathAndElevateWithinDist(path1R, ReefSide.BL, SnapButton.Right, 1.5, RequestState.CoralLevel3, e),
        AutoBuildingBlocks.autoStep("SCORE L3 RIGHT BL"),
        new LimelightAutoCmd(robot, ReefSide.BL, e, drivetrain, SnapButton.Right, RequestState.CoralLevel3, swerve, 2),
        AutoBuildingBlocks.autoStep("Return to Feeder"),
        AutoBuildingBlocks.followPathCommand(path2R),

        new WaitForCoralCmd(e),
        
        AutoBuildingBlocks.autoStep("PATH 1L"),
        new PathAndElevateWithinDist(path1L, ReefSide.BL, SnapButton.Left, 1.5, RequestState.CoralLevel2, e),
        AutoBuildingBlocks.autoStep("SCORE L3 Left BL"),
        new LimelightAutoCmd(robot, ReefSide.BL, e, drivetrain, SnapButton.Left, RequestState.CoralLevel2, swerve, 2),
        AutoBuildingBlocks.autoStep("Return to Feeder"),
        AutoBuildingBlocks.followPathCommand(path2L),

        new WaitForCoralCmd(e),
        
        AutoBuildingBlocks.autoStep("PATH 1R"),
        new PathAndElevateWithinDist(path1R, ReefSide.BL, SnapButton.Right, 1.5, RequestState.CoralLevel2, e),
        AutoBuildingBlocks.autoStep("SCORE L3 RIGHT BL"),
        new LimelightAutoCmd(robot, ReefSide.BL, e, drivetrain, SnapButton.Right, RequestState.CoralLevel2, swerve, 2),
        AutoBuildingBlocks.autoStep("Return to Feeder"),
        AutoBuildingBlocks.followPathCommand(path2R)
    );
  }

  @Override
  List<Pose2d> getAllRawPathPoses() {
    return Stream.of(path1L.getPathPoses(), path1R.getPathPoses(), path2L.getPathPoses(), path2R.getPathPoses())
        .flatMap(Collection::stream)
        .collect(Collectors.toList());
  }

  @Override
  public Pose2d getRawStartingPose() {
    return path1L.getStartingHolonomicPose().orElseThrow();
  }

}
