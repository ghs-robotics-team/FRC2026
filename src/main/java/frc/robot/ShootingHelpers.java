package frc.robot;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Helper functions for shooting calculations and Interpolation.
 */
public class ShootingHelpers {

  /**
   * Angle interpolation based on speaker position test data.
   * 
   * @param speakerPos Position of the speaker.
   * @return Target angle in degrees.
   */
  public static double angleInterp(Translation2d speakerPos) {
    double distance = speakerPos.minus(Globals.EagleEye.position.getTranslation()).getNorm();
    SmartDashboard.putNumber("dist", distance);
    Double[][] references = { { 0.0, 0.0 }, { 1.0, 1.0 } };
    try {
      references = Files.readAllLines(Paths.get("src", "main", "deploy", "shootingData.txt"))
          .stream()
          .map(line -> line.trim().split("\\s+"))
          .map(parts -> new double[] {
              Double.parseDouble(parts[0]),
              Double.parseDouble(parts[1])
          })
          .toList().toArray(Double[][]::new);
    } catch (IOException e) {
      e.printStackTrace();
    }

    // FOR TESTING
    // input angle, if angle works then write angle and distance
    // in references
    // Cant remember which variable we used for inputting the angle
    // Figure it out or create a different varible
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

    return targetDegree + 0.6;
  }

  /**
   * Gets the position of the speaker based on alliance color.
   * 
   * @param alliance Alliance color (Blue or Red).
   * @return Position of the speaker as a Translation2d.
   */
  public static Translation2d getSpeakerPos(Alliance alliance) {
    Translation2d speakerPos;
    if (alliance == Alliance.Blue) {
      speakerPos = new Translation2d(0.2286, 5.5826);
    } else {
      speakerPos = new Translation2d(16.256, 5.5826);
    }
    return speakerPos;
  }

  /**
   * Time interpolation based on speaker position test data to factor in
   * robot movement to the target position.
   * 
   * @param speakerPos Position of the speaker.
   * @return Target time in seconds.
   */
  public static double timeInterp(Translation2d speakerPos) {
    double distance = speakerPos.minus(Globals.EagleEye.position.getTranslation()).getNorm();
    SmartDashboard.putNumber("dist", distance);
    Double[][] references = { { 0.0, 0.0 }, { 1.0, 1.0 } }; // Get Time data
    try {
      references = Files.readAllLines(Paths.get("src", "main", "deploy", "shootingTimeData.txt"))
          .stream()
          .map(line -> line.trim().split("\\s+"))
          .map(parts -> new double[] {
              Double.parseDouble(parts[0]),
              Double.parseDouble(parts[1])
          })
          .toList().toArray(Double[][]::new);
    } catch (IOException e) {
      e.printStackTrace();
    }

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

  /**
   * Calculates the target position of the robot considering its velocity and time
   * to reach the speaker.
   * 
   * @return Target position of the robot.
   */
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