package frc.robot.commands.auto;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.drive.CommandSwerveDrivetrain;
import org.json.simple.parser.ParseException;

import java.io.IOException;

public class AutoBuildingBlocks {
  public static Command resetOdom(CommandSwerveDrivetrain drivetrain, PathPlannerPath path) {
    return new InstantCommand(() -> {
      Pose2d pose = path
          .getStartingHolonomicPose().orElseThrow();
      if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
        pose = FlippingUtil.flipFieldPose(pose);
      }

      drivetrain.resetPose(pose);
    });
  }
  
  public static Command autoStep(String step) {
    return new InstantCommand(() -> {
      SmartDashboard.putString("Auto Step", step);
    });
  }

  public static PathPlannerPath loadPathOrThrow(String pathname) {
    PathPlannerPath path;
    try {
      path = PathPlannerPath.fromPathFile(pathname);
      return path;
    } catch (FileVersionException | IOException | ParseException e1) {
      throw new IllegalArgumentException();
    }
  }

  public static CommandSwerveDrivetrain drivetrain;

  public static Command followPathCommand(PathPlannerPath path) {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (IOException | ParseException e1) {
      // TODO Auto-generated catch block
      throw new IllegalArgumentException();
    }
    return new FollowPathCommand(
        path,
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
    );
  }

}
