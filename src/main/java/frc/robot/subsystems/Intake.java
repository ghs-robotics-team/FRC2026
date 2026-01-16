// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  SparkMax IntakeMotor = new SparkMax(13, MotorType.kBrushless);
  //TalonFX alignmentMotor = new TalonFX(22);

  public Intake() {
  }

  public void spin(double speed) {
    IntakeMotor.set(speed);
    //alignmentMotor.setVoltage(6);
  }
  
  @Override
  public void periodic() {
  }
}
