package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (129) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = 6; // 0.2/2.0 Min/Max

  /**
   * Unit conversion constants.
   */
  public static final class UnitConversionConstants {
    public static final double INDEXER_TICKS_PER_DEGREES = 0.4; // To be calculated later
  }

  /**
   * Preset setpoints for the indexer arm.
   */
  public static final class SetPointConstants {
    public static final double ARM_AMP = -58.873;
    public static final double ARM_HUMAN_INTAKE = -50;
    public static final double ARM_INTAKE = 0;
    public static final double ARM_SPEAKER = -26.00; // -31.00;
    public static final double ARM_MAX = 0;
    public static final double ARM_MIN = -67.24;
    public static final double ARM_FAR = -42;
  }

  /**
   * Autonomous Drivetrain PID constants.
   */
  public static final class AutonConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(7.7, 0, 0.035); // D of 3?
    public static final PIDConstants ANGLE_PID = new PIDConstants(5.0, 0, 0.00);
  }

  /**
   * Drivebase constants.
   */
  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  /**
   * Operator interface constants.
   */
  public static class OperatorConstants {
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.05;
    public static final double LEFT_Y_DEADBAND = 0.05;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;

    // Makes data taking easier
    public static final boolean SHOOTING_DATA_COLLECTION_MODE = false;

    // Makes testing safer and easier
    public static final boolean WORKSHOP_MODE = false;

    // XBOX mode
    public static final boolean XBOX_DRIVE = false;
  }

  /**
   * Eagle Eye vision system constants.
   */
  public static class EagleEyeConstants {
    // m/s (Usually 1.5-2.5) before it stops reading vision measurements) WAS AT 2
    public static final double MAX_VISION_SPEED = 2.25;
    public static final boolean IN_PATH_END = false;
  }

  /**
   * Maximum temperature constants for motors.
   */
  public static class MaximumTemps {
    public static final double MaxNeoTemp = 90;
    public static final double MaxFalconTemp = 90;
  }
}