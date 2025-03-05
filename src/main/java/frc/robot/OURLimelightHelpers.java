package frc.robot;

import frc.robot.LimelightHelpers.RawFiducial;

public class OURLimelightHelpers {
  private static String curCam = "";

  public static double[][] getBotPoseTargetSpace(String camA, String camB, Double idToLookFor){
    return getBotPoseTargetSpace(camA, camB, idToLookFor, 1.5);
  }

  public static double[][] getBotPoseTargetSpace(String camA, String camB, Double idToLookFor, double maximumDistance) {
    if (curCam == "") {
      curCam = camA;
    }

    double tagIdA = 2200;
    double tagIdB = 2200;
    double[] idToRet = { 2200 };
    if (idToLookFor != null) {
      idToRet[0] = idToLookFor;
      tagIdA = idToLookFor;
      tagIdB = idToLookFor;
      //might not need to do every loop (keep in back pocket)
      int[] ids = { idToLookFor.intValue() };
      LimelightHelpers.SetFiducialIDFiltersOverride(camA, ids);
      LimelightHelpers.SetFiducialIDFiltersOverride(camB, ids);
    } else {
      tagIdA = LimelightHelpers.getFiducialID(camA);
      tagIdB = LimelightHelpers.getFiducialID(camB);
    }

    double[] botPoseA = LimelightHelpers.getBotPose_TargetSpace(camA);
    double[] botPoseB = LimelightHelpers.getBotPose_TargetSpace(camB);
    RawFiducial[] rawDataA = LimelightHelpers.getRawFiducials(camA);
    RawFiducial[] rawDataB = LimelightHelpers.getRawFiducials(camB);
    double ambA = 2200;
    double ambB = 2200;

    for (int i = 0; i < rawDataA.length; ++i) {
      if (rawDataA[i].id == tagIdA) {
        ambA = rawDataA[i].ambiguity;
        idToRet[0] = tagIdA;
        break;
      }
    }
    for (int i = 0; i < rawDataB.length; ++i) {
      if (rawDataB[i].id == tagIdB) {
        ambB = rawDataB[i].ambiguity;
        if (idToRet[0] == 2200) {
          idToRet[0] = tagIdB;
        }
        break;
      }
    }

    // If the *difference* in ambiguity is greater than X, we switch to lower amb
    // cam
    if (Math.abs(ambA - ambB) > 0.2) {
      curCam = (ambA < ambB) ? camA : camB;
    }

    if (ambB == 2200 && ambA == 2200) {
      return null;
    }
    double[][] ret = { {}, {} };
    if (curCam == camA) {
      ret[0] = botPoseA;
    } else {
      ret[0] = botPoseB;
    }
    // PYTHAGOREANS THEORUM BABYYYY
    double distanceToTarget = 0;
    if(ret[0] != null){
    distanceToTarget = Math.sqrt(Math.pow(-ret[0][2], 2) + Math.pow(ret[0][0], 2)); 
    }
    // SmartDashboard.putNumber("euclidian dist", distanceToTarget);
    if (distanceToTarget > maximumDistance && distanceToTarget != 0) { // distance to lock on, returns 0 if no tag so don't do that!

      return null; // TODO don't block????
    }
    ret[1] = idToRet;
    return ret;
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
