package frc.robot.commands.auto;

import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

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

public class Left4Piece extends AutoCommand {
  private final PathPlannerPath path1;
  private final PathPlannerPath path2;
  private final PathPlannerPath path3;
  private final PathPlannerPath path4;
  private final PathPlannerPath path5;
  private final PathPlannerPath path6;
  private final PathPlannerPath path7;

  public Left4Piece(RobotContainer robot, ElevClArmSubsystem e, CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric swerve) {
    path1 = AutoBuildingBlocks.loadPathOrThrow("4Left.1");
    path2 = AutoBuildingBlocks.loadPathOrThrow("4Left.2");
    path3 = AutoBuildingBlocks.loadPathOrThrow("4Left.3");
    path4 = AutoBuildingBlocks.loadPathOrThrow("4Left.4");
    path5 = AutoBuildingBlocks.loadPathOrThrow("4Left.5");
    path6 = AutoBuildingBlocks.loadPathOrThrow("4Left.6");
    path7 = AutoBuildingBlocks.loadPathOrThrow("4Left.7");
    addCommands(
        // is auto step the problem??
        new WaitCommand(0.01),
        AutoBuildingBlocks.autoStep("PATH 1"),
        Commands.parallel(
          AutoBuildingBlocks.followPathCommand(path1),
          Commands.sequence(
            new WaitCommand(0.7),
            new AutoRequesteStateCmd(e, RequestState.CoralLevel4)
          )
        ),
        // new PathAndElevateWithinDist(path1, ReefSide.BL, SnapButton.Right, 1.5, RequestState.CoralLevel4, e),
        AutoBuildingBlocks.autoStep("SCORE L4 RIGHT BL"),
        Commands.race(
          new GiveUp(e),
          new LimelightAutoCmd(robot, ReefSide.BL, e, drivetrain, SnapButton.Right, RequestState.CoralLevel4, swerve, 2)
        ),
        AutoBuildingBlocks.autoStep("PATH 2"),
        AutoBuildingBlocks.followPathCommand(path2),
        new WaitForCoralCmd(e),
        //new WaitCommand(0.01),
        AutoBuildingBlocks.autoStep("PATH 3"),
        new PathAndElevateWithinDist(path3, ReefSide.FL, SnapButton.Left, 1.5, RequestState.CoralLevel4, e),
        AutoBuildingBlocks.autoStep("SCORE L4 LEFT FL"),
        Commands.race(
          new GiveUp(e),
          new LimelightAutoCmd(robot, ReefSide.FL, e, drivetrain, SnapButton.Left, RequestState.CoralLevel4, swerve, 2)
        ),
        AutoBuildingBlocks.autoStep("PATH 4"),
        AutoBuildingBlocks.followPathCommand(path4),
        new WaitForCoralCmd(e),
        //new WaitCommand(0.01),
        AutoBuildingBlocks.autoStep("PATH 5"),
        new PathAndElevateWithinDist(path5, ReefSide.FL, SnapButton.Right, 1.5, RequestState.CoralLevel4, e),
        AutoBuildingBlocks.autoStep("3 PIECEEE BABY"),
        Commands.race(
          new GiveUp(e),
          new LimelightAutoCmd(robot, ReefSide.FL, e, drivetrain, SnapButton.Right, RequestState.CoralLevel4, swerve, 2)
        ),
        AutoBuildingBlocks.autoStep("PATH 6"),
        AutoBuildingBlocks.followPathCommand(path6),
        new WaitForCoralCmd(e),
        //new WaitCommand(0.01),
        AutoBuildingBlocks.autoStep("PATH 7"),
        Commands.parallel(
          AutoBuildingBlocks.followPathCommand(path7),
          new AutoRequesteStateCmd(e, RequestState.CoralLevel2)
        ),
        AutoBuildingBlocks.autoStep("Score Last"),
        Commands.race(
          new GiveUp(e),
          new LimelightAutoCmd(robot, ReefSide.FC, e, drivetrain, SnapButton.Left, RequestState.CoralLevel2, swerve, 2)
        ),
        AutoBuildingBlocks.autoStep("DONE")
    );
  }

  @Override
  List<Pose2d> getAllRawPathPoses() {
    return Stream.of(path1.getPathPoses(), path2.getPathPoses(), path3.getPathPoses(), path4.getPathPoses(), path5.getPathPoses(), path6.getPathPoses(), path7.getPathPoses())
        .flatMap(Collection::stream)
        .collect(Collectors.toList());
  }

  @Override
  public Pose2d getRawStartingPose() {
    return path1.getStartingHolonomicPose().orElseThrow();
  }

}
