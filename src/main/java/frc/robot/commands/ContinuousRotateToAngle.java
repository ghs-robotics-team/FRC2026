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

public class ContinuousRotateToAngle extends Command {
  /** Creates a new RotateToAngle. */
  SwerveSubsystem swerve;
  double degreeError;
  PIDController pid;
  double direction;
  double targetDegree;
  DoubleSupplier translationX;
  DoubleSupplier translationY;
  public boolean found;

  public ContinuousRotateToAngle(SwerveSubsystem swerve, DoubleSupplier translationX, DoubleSupplier translationY) {
    // Use addRequirements() here to declare subsystem dependencies.
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

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.drive(new Translation2d(0, 0), 0, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d speakerPos = ShootingHelpers.getTargetPos();
    /*if (DriverStation.getAlliance().get() == Alliance.Blue) {
      speakerPos = new Translation2d(0.02, 5.5826);
    } else {
      speakerPos = new Translation2d(16.4646, 5.5826);
    }*/

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

    // Set PID numbers
    pid.setP(P);
    pid.setI(I);
    pid.setD(D);

    SmartDashboard.putNumber("currentError", pid.getPositionError());
    direction = pid.calculate(swerve.getPose().getRotation().getDegrees(), targetDegree);

    if (direction < 0.06 && direction > -0.06) {
      direction = Math.copySign(0.06, direction);

    }

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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0, 0), 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
