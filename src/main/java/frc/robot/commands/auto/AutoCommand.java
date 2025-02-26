package frc.robot.commands.auto;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.List;

public abstract class AutoCommand extends SequentialCommandGroup {

  abstract List<Pose2d> getAllRawPathPoses();

  abstract Pose2d getRawStartingPose();


  public List<Pose2d> getAllProperFlippedPathPoses() {
    List<Pose2d> rawPoses = getAllRawPathPoses();
    return rawPoses.stream()
            .map(FlippingUtil::flipFieldPose)
            .collect(java.util.stream.Collectors.toList());
  }

  public Pose2d getProperFlippedStartingPose() {
    return FlippingUtil.flipFieldPose(getRawStartingPose());
  }
}
