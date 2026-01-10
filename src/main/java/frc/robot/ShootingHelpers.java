package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootingHelpers {
  public static double angleInterp(Translation2d speakerPos) {
    double distance = speakerPos.minus(Globals.EagleEye.position.getTranslation()).getNorm();
    SmartDashboard.putNumber("dist", distance);
    double[][] references = {
      {0.00,-38},
      {1.055,-38},
      {1.540,-35},
      {1.946,-33.5},
      {2.33,-32.5},
      {2.932,-31.25},
      {3.323,-32.0},
      {3.756,-29.75},
      {4.104,-29.65}
      /*{0,-38.25},
      {0.8,-34.5},
      {1.02, -33.6},
      {1.3, -33.2},
      {1.5,-32.2},
      {1.87,-31.125},
      {2.2, -30.7},
      {2.39,-30.1},
      {2.79,-29}*/
    }; // Distance,Degrees
    // FOR TESTING
    //input angle, if angle works then write angle and distance 
    //in references
    //Cant remember which variable we used for inputting the angle
    //Figure it out or create a different varible

    int index = -1;
    for (int i = 0; i < references.length; i++) {
      if (references[i][0] >= distance) {
        index = i - 1;
        break;
      }
    }
    if (index == -1) {
      return -1.0;
    }

    double interp = (distance - references[index][0]) / (references[index][0] - references[index + 1][0]);

    double targetDegree = (references[index][1] - references[index + 1][1]) * interp + references[index][1];

    return targetDegree+0.6;
  }

  public static Translation2d getSpeakerPos(Alliance alliance) {
    Translation2d speakerPos;
    if (alliance == Alliance.Blue) {
      speakerPos = new Translation2d(0.2286, 5.5826);
    } else {
      speakerPos = new Translation2d(16.256, 5.5826);
    }
    return speakerPos;
  }

  public static double timeInterp(Translation2d speakerPos) {
    double distance = speakerPos.minus(Globals.EagleEye.position.getTranslation()).getNorm();
    SmartDashboard.putNumber("dist", distance);
    double[][] references = { // Get Time Data
        {0, 0.15},
        {1.182,0.22},
        {1.955,0.28},
        {2.824,0.37},
        {3.765,0.51},
        {10,0.51}
    }; // Distance,Time
    

    int index = -1;
    for (int i = 0; i < references.length; i++) {
      if (references[i][0] >= distance) {
        index = i - 1;
        break;
      }
    }
    if (index == -1) {
      return -1.0;
    }

    double interp = (distance - references[index][0]) / (references[index][0] - references[index + 1][0]);

    double targetTime = (references[index][1] - references[index + 1][1]) * interp + references[index][1];

    return targetTime;
  }

  public static Translation2d getTargetPos() {
    // Distance, Interpolation of TIme, Velocity of Robot (eagle Eye)
    Translation2d speakerPos = getSpeakerPos(DriverStation.getAlliance().get());
    double xVel = Globals.EagleEye.xVel;
    double yVel = Globals.EagleEye.yVel;
    double time = timeInterp(speakerPos);
    Translation2d targetPos = new Translation2d(speakerPos.getX() - xVel * time, speakerPos.getY() - yVel * time);

    for (int i = 0; i < 4; i++) {
      time = timeInterp(targetPos);
      targetPos = new Translation2d(speakerPos.getX() - xVel * time, speakerPos.getY() - yVel * time);
    }

    return targetPos;
    
  }
}
