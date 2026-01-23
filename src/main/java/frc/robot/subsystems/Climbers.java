// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Climber subsystem for controlling the left and right climbers.
 */
public class Climbers extends SubsystemBase {
  private SparkMax LeftClimber = new SparkMax(20, MotorType.kBrushless);
  private SparkMax RightClimber = new SparkMax(19, MotorType.kBrushless);
  private double minLPos = -154.28;
  private double maxLPos = 5;
  private double minRPos = -117.31;
  private double maxRPos = 5;

  /**
   * Initializes motor configurations and SmartDashboard entries.
   */
  public Climbers() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    LeftClimber.configure(config, com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
    RightClimber.configure(config, com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
    SmartDashboard.putBoolean("Enable Limits", true);
    SmartDashboard.putBoolean("Zero Climbers", false);
  }

  /**
   * Sets the positions of the left and right climbers with limit checks.
   * 
   * @param posLeft  Left climber position.
   * @param posRight Right climber position.
   */
  public void setPos(double posLeft, double posRight) {
    if (posLeft < 0 && LeftClimber.getEncoder().getPosition() > minLPos
        || posLeft > 0 && LeftClimber.getEncoder().getPosition() < maxLPos
        || !SmartDashboard.getBoolean("Enable Limits", true)) {
      LeftClimber.set(Math.pow(posLeft, 3) / 1.8);
    } else {
      LeftClimber.set(0);
    }

    if (posRight < 0 && RightClimber.getEncoder().getPosition() > minRPos
        || posRight > 0 && RightClimber.getEncoder().getPosition() < maxRPos
        || !SmartDashboard.getBoolean("Enable Limits", true)) {
      RightClimber.set(Math.pow(posRight, 3) / 1.8);
    } else {
      RightClimber.set(0);
    }
    if (SmartDashboard.getBoolean("Zero Climbers", false)) {
      LeftClimber.getEncoder().setPosition(0);
      RightClimber.getEncoder().setPosition(0);
      SmartDashboard.putBoolean("Zero Climbers", false);
    }

    SmartDashboard.putNumber("left climber pos", LeftClimber.getEncoder().getPosition());
    SmartDashboard.putNumber("right climber pos", RightClimber.getEncoder().getPosition());
  }

  /**
   * Periodic method not used.
   */
  @Override
  public void periodic() {
  }
}