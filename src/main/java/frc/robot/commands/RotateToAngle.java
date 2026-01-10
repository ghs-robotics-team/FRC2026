// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateToAngle extends Command {
  /** Creates a new RotateToAngle. */
  SwerveSubsystem swerve;
  double degreeError;
  PIDController pid;
  double direction;
  double targetDegree;
  public boolean found;

  public RotateToAngle(SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
    this.swerve = swerve;
    this.pid = new PIDController(0.08, 0, 0.004); // set PID directions
    pid.enableContinuousInput(-180, 180);
    SmartDashboard.putNumber("PID-P", pid.getP());
    SmartDashboard.putNumber("PID-I", pid.getI());
    SmartDashboard.putNumber("PID-D", pid.getD());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.drive(new Translation2d(0, 0), 0, true);
    LimelightTarget_Fiducial[] target_Fiducials = LimelightHelpers
        .getLatestResults("limelight-april").targets_Fiducials;
    degreeError = 0;

    for (LimelightTarget_Fiducial target : target_Fiducials) {
      if (target.fiducialID == 4) {
        degreeError = target.tx;
      }
    }

    if (degreeError == 0) {
      Translation2d speakerPos;
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
        speakerPos = new Translation2d(0.02, 5.5826);
      } else {
        speakerPos = new Translation2d(16.4646, 5.5826);
      }
      degreeError = speakerPos.minus(swerve.getPose().getTranslation()).getAngle().getDegrees()
          - swerve.getPose().getRotation().getDegrees();
      degreeError = -degreeError;
      SmartDashboard.putNumber("distance", speakerPos.minus(swerve.getPose().getTranslation()).getNorm());
    }

    Translation2d speakerPos;
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      speakerPos = new Translation2d(0.02, 5.5826);
    } else {
      speakerPos = new Translation2d(16.4646, 5.5826);
    }

    SmartDashboard.putNumber("distance", speakerPos.minus(swerve.getPose().getTranslation()).getNorm());
    targetDegree = swerve.getPose().getRotation().getDegrees() - degreeError;

    if (targetDegree > 180) {
      targetDegree -= 360;
    } else if (targetDegree < -180) {
      targetDegree += 360;
    }

    SmartDashboard.putNumber("targetDegree", targetDegree);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
    if (pid.getPositionError() > -0.25 && pid.getPositionError() < 0.25) {
      swerve.drive(new Translation2d(0, 0), 0, true);
      // deadzone
    } else {
      swerve.drive(new Translation2d(0, 0), direction, true);
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
