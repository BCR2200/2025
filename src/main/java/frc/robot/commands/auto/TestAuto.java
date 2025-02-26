package frc.robot.commands.auto;

import java.io.IOException;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.input.SnapButton;
import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem.RequestState;

public class TestAuto extends AutoCommand {
  private final PathPlannerPath path1;

  public TestAuto(ElevClArmSubsystem e, CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric swerve) {
    path1 = AutoBuildingBlocks.loadPathOrThrow("testing testy path");
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (IOException | ParseException e1) {
      // TODO Auto-generated catch block
      e1.printStackTrace();
      throw new IllegalArgumentException();
    }
    addCommands(

        AutoBuildingBlocks.resetOdom(drivetrain, path1),
        Commands.sequence(
            AutoBuildingBlocks.autoStep("FOLLOW"),

            new FollowPathCommand(
                path1,
                () -> drivetrain.getState().Pose,
                () -> drivetrain.getState().Speeds,
                (speeds, feedforwards) -> drivetrain.setControl(
                    drivetrain.m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(5, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7.0, 0, 0)),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the
                // case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                drivetrain // Subsystem for requirements
            ),

            AutoBuildingBlocks.autoStep("GO UP"),
            new LimelightAutoCmd(e, drivetrain, SnapButton.Left, RequestState.CoralLevel4, swerve, 2)),
        AutoBuildingBlocks.autoStep("DONE"));

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
