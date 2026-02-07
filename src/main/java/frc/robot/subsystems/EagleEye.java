// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EagleEyeConstants;
import frc.robot.Globals;
import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.PoseEstimate;

/**
 * EagleEye subsystem for vision processing and pose estimation.
 */
public class EagleEye extends SubsystemBase {
  // Cameras
  private final Limelight cameraA = new Limelight("limelight-cama");
  private final Limelight cameraB = new Limelight("limelight-camb");

  // Get pose estimates
  LimelightPoseEstimator estimatorA = cameraA.createPoseEstimator(EstimationMode.MEGATAG2);

  LimelightPoseEstimator estimatorB = cameraB.createPoseEstimator(EstimationMode.MEGATAG2);

  /**
   * Nothing done in init.
   */
  public EagleEye() {
  }

  /**
   * Determines confidence level of vision estimations
   * based on distance, tag count, and robot motion.
   * 
   * @param estimate Optional PoseEstimate from a camera
   * @return confidence level (0 to 1) of the vision measurement
   */
  public double limelightMeasurement(Optional<PoseEstimate> estimate) {
    if (estimate.isEmpty())
      return 0.0;

    PoseEstimate est = estimate.get();

    // Hard rejections
    if (est.tagCount < 1)
      return 0.0;
    if (est.avgTagDist > Units.feetToMeters(20))
      return 0.0;
    if (Math.abs(Globals.EagleEye.rotVel) > Math.PI / 2)
      return 0.0;
    if (Math.hypot(Globals.EagleEye.xVel, Globals.EagleEye.yVel) > EagleEyeConstants.MAX_VISION_SPEED)
      return 0.0;

    // Start with full trust
    double confidence = 1.0;

    // Distance Penalties
    double distMeters = est.avgTagDist;
    double distFactor = MathUtil.clamp(1.0 - distMeters / Units.feetToMeters(20), 0.0, 1.0);
    confidence *= distFactor;

    // Tag Count Bonuses
    if (est.tagCount >= 2) {
      confidence *= 1.0; // keep
    } else {
      confidence *= 0.5; // single-tag penalty
    }

    // Compare with Odometry
    double odomError = est.pose.getTranslation()
        .getDistance(Globals.EagleEye.position.getTranslation());

    double odomFactor = MathUtil.clamp(1.0 - odomError / 1.0, 0.0, 1.0); // 1m tolerance
    confidence *= odomFactor;

    // Disabled or stationary bonus
    if (DriverStation.isDisabled()) {
      confidence *= 1.2;
    }

    // Clamp final confidence to [0, 1] to avoid invalid values.
    return MathUtil.clamp(confidence, 0.0, 1.0);
  }

  /**
   * Periodic update for vision processing.
   */
  @Override
  public void periodic() {
    if (RobotBase.isSimulation())
      return;

    double confidenceA = 0;
    double confidenceB = 0;

    Optional<PoseEstimate> estimateA = estimatorA.getPoseEstimate();
    Optional<PoseEstimate> estimateB = estimatorB.getPoseEstimate();

    /**
     * LimelightHelpers.SetRobotOrientation("limelight-camb",
     * Globals.EagleEye.rawGyroYaw, 0, 0,
     * 0, 0, 0);
     * 
     * LimelightHelpers.SetRobotOrientation("limelight-cama",
     * Globals.EagleEye.rawGyroYaw, 0, 0,
     * 0, 0, 0);
     **/

    // Feed RAW gyro yaw to vision
    cameraA.getNTTable()
        .getEntry("robot_orientation")
        .setDoubleArray(new double[] {
            Globals.EagleEye.rawGyroYaw, // yaw (deg)
            0, 0, 0, 0, 0
        });

    cameraB.getNTTable()
        .getEntry("robot_orientation")
        .setDoubleArray(new double[] {
            Globals.EagleEye.rawGyroYaw,
            0, 0, 0, 0, 0
        });

    // Camera A
    if (estimateA.isPresent()) {
      SmartDashboard.putNumber("EEA NumTags", estimateA.get().tagCount);
      SmartDashboard.putNumber("EEA Avg Tag Dist", estimateA.get().avgTagDist);
      SmartDashboard.putNumber("EE Rotation Vel", Globals.EagleEye.rotVel);
      SmartDashboard.putNumber(
          "EE Total Vel",
          Math.hypot(Globals.EagleEye.xVel, Globals.EagleEye.yVel));

      // If alliance is RED, mirror the pose returned in Blue coordinates into Red
      // field coordinates. This mirrors the X coordinate about the field length
      // so fused poses are consistent with the robot's alliance frame.
      var poseA = estimateA.get().pose;
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        final double FIELD_LENGTH = 16.4646; // meters (used elsewhere in code)
        double newX = FIELD_LENGTH - poseA.getTranslation().getX();
        double newY = poseA.getTranslation().getY();
        // Rotate yaw by 180 degrees
        var rot = poseA.getRotation();
        var newRot = rot.plus(new edu.wpi.first.math.geometry.Rotation3d(0, 0, Math.toRadians(180)));
        poseA = new edu.wpi.first.math.geometry.Pose3d(
            new edu.wpi.first.math.geometry.Translation3d(newX, newY, poseA.getTranslation().getZ()), newRot);
      }

      // Recreate an Optional-like PoseEstimate with the possibly mirrored pose
      PoseEstimate updatedA = estimateA.get();
      updatedA.pose = poseA;

      confidenceA = limelightMeasurement(java.util.Optional.of(updatedA));

      Globals.LastVisionMeasurement.positionA = updatedA.pose;
      Globals.LastVisionMeasurement.timeStampA = updatedA.timestampSeconds;
      Globals.LastVisionMeasurement.notRead = true;
    }

    // Camera B
    if (estimateB.isPresent()) {
      SmartDashboard.putNumber("EEB NumTags", estimateB.get().tagCount);
      SmartDashboard.putNumber("EEB Avg Tag Dist", estimateB.get().avgTagDist);
      SmartDashboard.putNumber("EE Rotation Vel", Globals.EagleEye.rotVel);
      SmartDashboard.putNumber(
          "EE Total Vel",
          Math.hypot(Globals.EagleEye.xVel, Globals.EagleEye.yVel));

      var poseB = estimateB.get().pose;
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        final double FIELD_LENGTH = 16.4646;
        double newX = FIELD_LENGTH - poseB.getTranslation().getX();
        double newY = poseB.getTranslation().getY();
        var rot = poseB.getRotation();
        var newRot = rot.plus(new edu.wpi.first.math.geometry.Rotation3d(0, 0, Math.toRadians(180)));
        poseB = new edu.wpi.first.math.geometry.Pose3d(
            new edu.wpi.first.math.geometry.Translation3d(newX, newY, poseB.getTranslation().getZ()), newRot);
      }
      PoseEstimate updatedB = estimateB.get();
      updatedB.pose = poseB;

      confidenceB = limelightMeasurement(java.util.Optional.of(updatedB));

      Globals.LastVisionMeasurement.positionB = updatedB.pose;
      Globals.LastVisionMeasurement.timeStampB = updatedB.timestampSeconds;
      Globals.LastVisionMeasurement.notRead = true;
    }

    Globals.LastVisionMeasurement.confidenceA = confidenceA;
    Globals.LastVisionMeasurement.confidenceB = confidenceB;

    // Shooting data collection (unchanged)
    if (Constants.OperatorConstants.SHOOTING_DATA_COLLECTION_MODE) {

      if (SmartDashboard.getBoolean("Record Data", false)) {
        File file = new File(
            Paths.get("src", "main", "deploy", "shootingData.txt").toUri());
        try (FileWriter writer = new FileWriter(file)) {
          writer.write(
              SmartDashboard.getNumber("dist", 0) + "  "
                  + SmartDashboard.getNumber("Test Angle", 0) + "\n");
        } catch (IOException e) {
          e.printStackTrace();
        }
        SmartDashboard.putBoolean("Record Data", false);
      }

      boolean button = SmartDashboard.getBoolean("Record Time Data", false);
      Timer timer = Globals.shootingDataCollectionSettings.timer;

      if (button && !Globals.shootingDataCollectionSettings.lastButtonState) {

        if (!Globals.shootingDataCollectionSettings.recording) {
          timer.reset();
          timer.start();
          Globals.shootingDataCollectionSettings.recording = true;
          Globals.shootingDataCollectionSettings.startPose = Globals.EagleEye.position;

        } else {
          timer.stop();

          double elapsedTime = timer.get();
          Globals.shootingDataCollectionSettings.endPose = Globals.EagleEye.position;

          double distance = Globals.shootingDataCollectionSettings.endPose
              .getTranslation()
              .getDistance(
                  Globals.shootingDataCollectionSettings.startPose
                      .getTranslation());

          File file = new File(
              Paths.get("src", "main", "deploy", "shootingTimeData.txt").toUri());
          try (FileWriter writer = new FileWriter(file)) {
            writer.write(distance + "  " + elapsedTime + "\n");
          } catch (IOException e) {
            e.printStackTrace();
          }

          Globals.shootingDataCollectionSettings.recording = false;
        }

        SmartDashboard.putBoolean("Record Time Data", false);
      }

      Globals.shootingDataCollectionSettings.lastButtonState = button;
    }
  }
}