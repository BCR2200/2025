package frc.robot.commands.auto;

import java.io.IOException;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.LimelightAutoCmd;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.input.SnapButton;
import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem.RequestState;

public class RightTwoLvl4 extends AutoCommand {
  private final PathPlannerPath path1;

  public RightTwoLvl4(ElevClArmSubsystem e, CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric swerve) {
    try {
      path1 = PathPlannerPath.fromPathFile("testing testy path");
    } catch (FileVersionException | IOException | ParseException e1) {
      throw new IllegalArgumentException();
    }

    addCommands(
        Commands.deadline(
            Commands.sequence(
                AutoBuilder.followPath(path1),
                new LimelightAutoCmd(e, drivetrain, SnapButton.Left, RequestState.CoralLevel4, swerve, 100000000)
            )
        )
    );

  }

  @Override
  public List<Pose2d> getAllPathPoses() {
    return Stream.of(
        path1.getPathPoses())
        .flatMap(Collection::stream)
        .collect(Collectors.toList());
  }

  @Override
  public Pose2d getStartingPose() {
    return path1.getStartingHolonomicPose().orElseThrow();
  }

}
