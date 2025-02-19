package frc.robot;

public class OURLimelightHelpers {
    
    
  public static double[] getValidBotPose(String primary, String fallback) {
    double[] botPose = LimelightHelpers.getBotPose_TargetSpace(primary);
    if (doesBotPoseExist(botPose)) {
      return botPose;
    }
    botPose = LimelightHelpers.getBotPose_TargetSpace(fallback);
    if (doesBotPoseExist(botPose)) {
      return botPose;
    }
    return null;
  }

  public static boolean doesBotPoseExist(double[] botPose) {
    for (double value : botPose) {
      if (value != 0.0) {
        return true;
      }
    }
    return false;
  }
}
