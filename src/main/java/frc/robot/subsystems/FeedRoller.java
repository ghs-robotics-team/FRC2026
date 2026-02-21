// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;

public class FeedRoller extends SubsystemBase {
  SparkFlex rollerMotor = new SparkFlex(19, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
  /** Creates a new FeedRoller. */
  public FeedRoller() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void roll (double power) {
    rollerMotor.set(power);
  }
}
