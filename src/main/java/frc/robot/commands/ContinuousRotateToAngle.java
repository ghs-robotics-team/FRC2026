// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Globals;
import frc.robot.ShootingHelpers;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveDrive;

/**
 * Continously rotates the robot to face the game piece scoring location.
 */
public class ContinuousRotateToAngle extends Command {
  private SwerveSubsystem swerve;
  private double degreeError;
  private PIDController pid;
  private double direction;
  private double targetDegree;
  private DoubleSupplier translationX;
  private DoubleSupplier translationY;

  /**
   * Initializes the command and establishes command PID.
   * 
   * @param swerve       Swerve system to use.
   * @param translationX x position on the field of the robot.
   * @param translationY y position on the field of the robot.
   */
  public ContinuousRotateToAngle(SwerveSubsystem swerve, DoubleSupplier translationX, DoubleSupplier translationY) {
    addRequirements(swerve);
    this.swerve = swerve;
    this.pid = new PIDController(0.08, 0, 0.004); // set PID directions
    this.translationX = translationX;
    this.translationY = translationY;
    pid.enableContinuousInput(-180, 180);

    SmartDashboard.putNumber("PID-P", pid.getP());
    SmartDashboard.putNumber("PID-I", pid.getI());
    SmartDashboard.putNumber("PID-D", pid.getD());
  }

  /**
   * Stops the robot from moving at the start of the command.
   */
  @Override
  public void initialize() {
    swerve.drive(new Translation2d(0, 0), 0, true);
  }

  /**
   * Continously calculates the angle error and moves the robot to face the target
   * position.
   */
  @Override
  public void execute() {
    Translation2d speakerPos = ShootingHelpers.getTargetPos();

    /*
     * (Outdated code, might be used later)
     * if (DriverStation.getAlliance().get() == Alliance.Blue) {
     * speakerPos = new Translation2d(0.02, 5.5826);
     * } else {
     * speakerPos = new Translation2d(16.4646, 5.5826);
     * }
     */

    // Calculates the angle error between the robot and the target position.
    degreeError = speakerPos.minus(Globals.EagleEye.position.getTranslation()).getAngle().getDegrees()
        - Globals.EagleEye.position.getRotation().getDegrees();
    degreeError = -degreeError;
    SmartDashboard.putNumber("distance", speakerPos.minus(Globals.EagleEye.position.getTranslation()).getNorm());
    targetDegree = Globals.EagleEye.position.getRotation().getDegrees() - degreeError;

    if (targetDegree > 180) {
      targetDegree -= 360;
    } else if (targetDegree < -180) {
      targetDegree += 360;
    }

    SmartDashboard.putNumber("targetDegree", targetDegree);

    double P = SmartDashboard.getNumber("PID-P", 1.0 / 150.0);
    double I = SmartDashboard.getNumber("PID-I", 0.0);
    double D = SmartDashboard.getNumber("PID-D", 0);

    pid.setP(P);
    pid.setI(I);
    pid.setD(D);

    SmartDashboard.putNumber("currentError", pid.getPositionError());

    // Calculates power using PID to move the motors to the target angle.
    direction = pid.calculate(swerve.getPose().getRotation().getDegrees(), targetDegree);

    if (direction < 0.06 && direction > -0.06) {
      direction = Math.copySign(0.06, direction);
    }

    // Fits direction into -4 to 4 range for swerve drive.
    direction = MathUtil.clamp(direction, -4, 4);
    SmartDashboard.putNumber("Old direction", direction);
    SwerveDrive swerveDrive = swerve.getSwerveDrive();
    double xIn = 0;
    double yIn = 0;

    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      xIn = -translationX.getAsDouble();
      yIn = -translationY.getAsDouble();
    } else {
      xIn = translationX.getAsDouble();
      yIn = translationY.getAsDouble();
    }

    if (pid.getPositionError() > -0.25 && pid.getPositionError() < 0.25) {
      swerve.drive(new Translation2d(Math.pow(xIn, 3) * swerveDrive.getMaximumChassisVelocity(),
          Math.pow(yIn, 3) * swerveDrive.getMaximumChassisVelocity()), 0, true);
      // deadzone
    } else {
      swerve.drive(new Translation2d(Math.pow(xIn, 3) * swerveDrive.getMaximumChassisVelocity(),
          Math.pow(yIn, 3) * swerveDrive.getMaximumChassisVelocity()), direction, true);
    }
  }

  /**
   * Stops the robot from moving when the command ends.
   * 
   * @param interrupted whether the command was interrupted.
   */
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0, 0), 0, true);
  }

  /**
   * Indicates that the command never finishes on its own.
   * 
   * @return false always.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
