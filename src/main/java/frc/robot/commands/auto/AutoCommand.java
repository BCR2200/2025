package frc.robot.commands.auto;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

import java.util.List;

public abstract class AutoCommand extends SequentialCommandGroup {

  abstract List<Pose2d> getAllRawPathPoses();

  abstract Pose2d getRawStartingPose();


  public List<Pose2d> getAllProperFlippedPathPoses() {
    List<Pose2d> rawPoses = getAllRawPathPoses();
    boolean isOnRed = Robot.alliance == Alliance.Red;
    return rawPoses.stream()
        .map((Pose2d pose) -> {
          if (isOnRed) {
            return FlippingUtil.flipFieldPose(pose);
          }
          return pose;
        })
        .collect(java.util.stream.Collectors.toList());
  }

  public Pose2d getProperFlippedStartingPose() {
    boolean isOnRed = Robot.alliance == Alliance.Red;
    Pose2d rawStartingPose = getRawStartingPose();
    if (isOnRed) {
      return FlippingUtil.flipFieldPose(getRawStartingPose());
    }
    return rawStartingPose;
  }
}
