// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Shoots game pieces at end of subsystem chain. 
 * Uses a single motor to shoot and a hood angler to adjust the angle of the shot.
 */
public class Shooter extends SubsystemBase {
  TalonFX shooter = new TalonFX(18);

  /**
   * Nothing done in constructor.
   */
  public Shooter() {}

  /**
   * Sets the power level of the shooter motor to shoot game pieces.
   * @param power The power level to set the shooter motor to, typically between -1.0 and 1.0.
   */
  public void shoot(double power) {
    shooter.set(power);
  }

  /*
   * Periodic is not used.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
