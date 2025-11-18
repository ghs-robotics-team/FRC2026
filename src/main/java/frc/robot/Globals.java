package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;

public final class Globals {
  public static class EagleEye {
    public static Pose2d position = new Pose2d();
    public static double xVel = 0;
    public static double yVel = 0;
    public static double rotVel = 0;
  }

  public static class LastVisionMeasurement {
    public static Pose2d positiona = new Pose2d();
    public static Pose2d positionb = new Pose2d();
    public static double timeStamp = 0;
    public static boolean notRead = false;
    public static double confidencea = 0;
    public static double confidenceb = 0;
  }

  public static class LastTempMeasurement {
    public static double topShooterTemp = 0;
    public static double bottomShooterTemp = 0;
  }
  
  public static class targetPos {
    public static double armTarget = 0;
    public static double elevatorTarget = 0;
  }

  public static boolean inPath = false;
  public static XboxController buttonsXbox;
  public static double inversion = 1;
}
