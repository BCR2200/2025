package frc.robot.commands.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.drive.CommandSwerveDrivetrain;

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


}
