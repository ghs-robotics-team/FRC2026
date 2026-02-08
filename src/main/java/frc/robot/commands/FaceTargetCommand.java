package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class FaceTargetCommand extends Command {
  private static final String kExecutingKey = "FT Executing";
  private static final String kRotatingKey = "FT Rotating";
  private static final String kTargetAngleKey = "FT TargetDeg";
  private static final String kErrorAngleKey = "FT ErrorDeg";
  private static final String kPKey = "FT P";
  private static final String kDKey = "FT D";
  private static final double kPDefault = 0.3;
  private static final double kDDefault = 0.01;
  private static final double kPMin = 0.0;
  private static final double kPMax = 5.0;
  private static final double kDMin = 0.0;
  private static final double kDMax = 0.5;
  private static final double kI = 0.0;
  private static final double kAngleToleranceDeg = 2.0;
  private static final double kMinAngularVelocityRadPerSec = 0.3;

  private static double cachedP = kPDefault;
  private static double cachedD = kDDefault;

  static {
    SmartDashboard.putBoolean(kExecutingKey, false);
    SmartDashboard.putBoolean(kRotatingKey, false);
    SmartDashboard.putNumber(kTargetAngleKey, 0.0);
    SmartDashboard.putNumber(kErrorAngleKey, 0.0);
    SmartDashboard.putNumber(kPKey, kPDefault);
    SmartDashboard.putNumber(kDKey, kDDefault);
  }

  private final SwerveSubsystem swerve;
  private final Pose2d target;
  private final PIDController rotationController;
  private final double maxAngularVelocity;

  public FaceTargetCommand(SwerveSubsystem swerve, Pose2d target) {
    this.swerve = swerve;
    this.target = target;
    this.rotationController = new PIDController(cachedP, kI, cachedD);
    this.rotationController.enableContinuousInput(-180.0, 180.0);
    this.rotationController.setTolerance(kAngleToleranceDeg);
    this.maxAngularVelocity = swerve.getSwerveDrive().getMaximumChassisAngularVelocity();
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean(kExecutingKey, true);
    refreshTunables();
    rotationController.setP(cachedP);
    rotationController.setD(cachedD);
    rotationController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = swerve.getPose();
    Translation2d toTarget = target.getTranslation().minus(currentPose.getTranslation());

    if (toTarget.getNorm() < 1e-4) {
      swerve.stop();
      return;
    }

    double targetAngle = toTarget.getAngle().getDegrees();
  double currentAngle = currentPose.getRotation().getDegrees();
  double errorAngle = MathUtil.inputModulus(targetAngle - currentAngle, -180.0, 180.0);

  SmartDashboard.putNumber(kTargetAngleKey, targetAngle);
  SmartDashboard.putNumber(kErrorAngleKey, errorAngle);

    double rotationOutputDegPerSec = rotationController.calculate(currentPose.getRotation().getDegrees(),
        targetAngle);
    double rotationOutput = Units.degreesToRadians(rotationOutputDegPerSec);
    rotationOutput = MathUtil.clamp(rotationOutput, -maxAngularVelocity, maxAngularVelocity);

    boolean rotating = true;
    if (rotationController.atSetpoint()) {
      rotationOutput = 0.0;
      rotating = false;
    } else if (Math.abs(rotationOutput) < kMinAngularVelocityRadPerSec) {
      rotationOutput = Math.copySign(kMinAngularVelocityRadPerSec, rotationOutput);
    }

    SmartDashboard.putBoolean(kRotatingKey, rotating);

    swerve.drive(new Translation2d(), rotationOutput, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stop();
    SmartDashboard.putBoolean(kExecutingKey, false);
    SmartDashboard.putBoolean(kRotatingKey, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private static void refreshTunables() {
    double p = SmartDashboard.getNumber(kPKey, kPDefault);
    double d = SmartDashboard.getNumber(kDKey, kDDefault);
    cachedP = MathUtil.clamp(p, kPMin, kPMax);
    cachedD = MathUtil.clamp(d, kDMin, kDMax);
  }
}
