// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Spindexer subsystem for indexing and feeding game pieces to the feed roller and shooter.
 */
public class Spindexer extends SubsystemBase {
  TalonFX indexer = new TalonFX(16);

  /**
   * Nothing done in constructor.
   */
  public Spindexer() {}

  /**
   * Sets the power level of the indexer motor to run the spindexer.
   * @param power The power level to set the indexer motor to, typically between -1.0 and 1.0.
   */
  public void run(double power) {
    indexer.set(power);
  }

  /**
   * Periodic is not used.
   */
  @Override
  public void periodic() {
  }
}
