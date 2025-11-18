package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Globals;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.EagleEyeConstants;

public class EagleEye extends SubsystemBase {
  /** Creates a new EagleEye. */
  public EagleEye() {
  }

  public double limelightMeasurement (LimelightHelpers.PoseEstimate limelight) {
    double confidence = 0;
    if (limelight.tagCount >= 1/* && fieldBoundary.isPoseWithinArea(limelightMeasurementa.pose) */) {
      // Excluding different measurements that are absolute showstoppers even with
      // full trust
      if (limelight.avgTagDist < Units.feetToMeters(15) && Globals.EagleEye.rotVel < Math.PI
          && Math.hypot(Globals.EagleEye.xVel, Globals.EagleEye.yVel) < EagleEyeConstants.MAX_VISION_SPEED) {
        // Reasons to blindly trust as much as odometry
        if (DriverStation.isDisabled() ||
            (limelight.tagCount >= 2 && limelight.avgTagDist < Units.feetToMeters(10))) {
          confidence = 0.2;
        } else {
          // High trust level anything less than this we shouldn't bother with
          double compareDistance = limelight.pose.getTranslation().getDistance(Globals.EagleEye.position.getTranslation());
            if( compareDistance < 0.5 ||
              (limelight.tagCount >= 2 && limelight.avgTagDist < Units.feetToMeters(20)) ||
              (limelight.tagCount == 1 && limelight.avgTagDist < Units.feetToMeters(15))) {
              double tagDistance = Units.metersToFeet(limelight.avgTagDist);
              // Double the distance for solo tag
              if (limelight.tagCount == 1) {
                tagDistance = tagDistance * 2;
              }
              // Add up to .2 confidence depending on how far away
              confidence = 0.7 + (tagDistance / 100);
            }
        }
      }
    }
    return confidence;
  }

  @SuppressWarnings("unused")
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (RobotBase.isSimulation())
      return;

    // Don't Read Eagleye during Teleop Paths 
    /*if(Constants.EagleEyeConstants.IN_PATH_END && Globals.inPath){ 
      Globals.LastVisionMeasurement.confidencea = 0; 
      Globals.LastVisionMeasurement.confidenceb = 0;
      return;
    } */

    // If we don't update confidence then we don't send the measurement
    double confidencea = 0;
    double confidenceb = 0;

    // Gets robot orientation from Gyro
    LimelightHelpers.SetRobotOrientation("limelight-camb", Globals.EagleEye.position.getRotation().getDegrees(), 0, 0,
        0, 0, 0);
    
    LimelightHelpers.SetRobotOrientation("limelight-cama", Globals.EagleEye.position.getRotation().getDegrees(), 0, 0,
        0, 0, 0);
    
    
    // Gets predicted location based on Tag
    LimelightHelpers.PoseEstimate limelightMeasurementa = LimelightHelpers
        .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-cama");

    LimelightHelpers.PoseEstimate limelightMeasurementb = LimelightHelpers
        .getBotPoseEstimate_wpiBlue_MegaTag2("limelight-camb");

    if (limelightMeasurementa != null){
      SmartDashboard.putNumber("EEA NumTags", limelightMeasurementa.tagCount);
      SmartDashboard.putNumber("EEA Avg Tag Dist", limelightMeasurementa.avgTagDist);
      SmartDashboard.putNumber("EE Rotation Vel", Globals.EagleEye.rotVel);
      SmartDashboard.putNumber("EE Total Vel", Math.hypot(Globals.EagleEye.xVel, Globals.EagleEye.yVel));

      confidencea = limelightMeasurement(limelightMeasurementa);

      Globals.LastVisionMeasurement.positiona = limelightMeasurementa.pose;
      Globals.LastVisionMeasurement.timeStamp = limelightMeasurementa.timestampSeconds;
      Globals.LastVisionMeasurement.notRead = true;
      
    }
    if (limelightMeasurementb != null) {
      
      SmartDashboard.putNumber("EEB NumTags", limelightMeasurementb.tagCount);
      SmartDashboard.putNumber("EEB Avg Tag Dist", limelightMeasurementb.avgTagDist);
      SmartDashboard.putNumber("EE Rotation Vel", Globals.EagleEye.rotVel);
      SmartDashboard.putNumber("EE Total Vel", Math.hypot(Globals.EagleEye.xVel, Globals.EagleEye.yVel));
    
      confidenceb = limelightMeasurement(limelightMeasurementb);

      // No tag found so check no further or pose not within field boundary
      Globals.LastVisionMeasurement.positionb = limelightMeasurementb.pose;
      Globals.LastVisionMeasurement.timeStamp = limelightMeasurementb.timestampSeconds;
      Globals.LastVisionMeasurement.notRead = true;
      
    }
    Globals.LastVisionMeasurement.confidencea = confidencea;
    Globals.LastVisionMeasurement.confidenceb = confidenceb;
  }
}
