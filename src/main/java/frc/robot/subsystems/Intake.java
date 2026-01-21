// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Intake subsystem for controlling the intake mechanism.
 */
public class Intake extends SubsystemBase {
  private SparkMax IntakeMotor = new SparkMax(13, MotorType.kBrushless);

  /**
   * Nothing done in init.
   */
  public Intake() {
  }

  /**
   * Spins the intake motor at the specified speed.
   * (Negative for one direction, positive for the other)
   * 
   * @param speed
   */
  public void spin(double speed) {
    IntakeMotor.set(speed);
  }

  /**
   * Nothing done periodically.
   */
  @Override
  public void periodic() {
  }
}