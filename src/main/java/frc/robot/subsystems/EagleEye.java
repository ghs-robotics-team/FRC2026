// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.Optional;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.EagleEyeConstants;
import frc.robot.Globals;
import frc.robot.LimelightHelpers;
// YALL imports
import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.PoseEstimate;

/**
 * EagleEye subsystem for vision processing and pose estimation.
 */
public class EagleEye extends SubsystemBase {

  // =====================
  // Cameras
  // =====================
  private final Limelight cameraA = new Limelight("limelight-cama");
  private final Limelight cameraB = new Limelight("limelight-camb");

  // =====================
  // Get pose estimates
  // =====================
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
   */
  public double limelightMeasurement(Optional<PoseEstimate> estimate) {
    double confidence = 0;

    if (estimate.isPresent() && estimate.get().tagCount >= 1) {
      PoseEstimate est = estimate.get();

      // Absolute rejection conditions
      if (est.avgTagDist < Units.feetToMeters(15)
          && Globals.EagleEye.rotVel < Math.PI / 2
          && Math.hypot(Globals.EagleEye.xVel, Globals.EagleEye.yVel) < EagleEyeConstants.MAX_VISION_SPEED) {

        // Near-perfect cases
        if (DriverStation.isDisabled()
            || (estimate.isPresent() && estimate.get().tagCount >= 2
                && estimate.get().avgTagDist < Units.feetToMeters(10))) {

          confidence = 0.2;

        } else {
          double compareDistanceMeters = est.pose.getTranslation()
              .getDistance(
                  Globals.EagleEye.position.getTranslation());

          if (compareDistanceMeters < 0.5
              || (estimate.get().tagCount >= 2
                  && estimate.get().avgTagDist < Units.feetToMeters(20))
              || (estimate.get().tagCount == 1
                  && estimate.get().avgTagDist < Units.feetToMeters(15))) {

            double tagDistanceFeet = Units.metersToFeet(estimate.get().avgTagDist);

            // Penalize single-tag harder
            if (estimate.get().tagCount == 1) {
              tagDistanceFeet *= 2;
            }

            confidence = 0.7 - (tagDistanceFeet / 100.0);
          }
        }
      }
    }
    return confidence;
  }

  /**
   * Periodic update for vision processing.
   */
  @Override
  public void periodic() {
    if (RobotBase.isSimulation())
      return;

    // Feed almost (YAGSL config like 'invertIMU' does affect this) raw gyro heading
    // to Limelights
    LimelightHelpers.SetRobotOrientation("limelight-camb", Globals.EagleEye.rawGyroYaw, 0, 0,
        0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-cama", Globals.EagleEye.rawGyroYaw, 0, 0,
        0, 0, 0);

    double confidenceA = 0;
    double confidenceB = 0;

    Optional<PoseEstimate> estimateA = estimatorA.getPoseEstimate();
    Optional<PoseEstimate> estimateB = estimatorB.getPoseEstimate();

    // =====================
    // Camera A
    // =====================
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

    // =====================
    // Camera B
    // =====================
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

    // =====================
    // Shooting data collection (unchanged)
    // =====================
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
