// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * HoodAngler subsystem for adjusting the angle of the shooter hood.
 * Effecting where the balls are angled when shot.
 */
public class HoodAngler extends SubsystemBase {
  SparkMax hoodAngler = new SparkMax(14, MotorType.kBrushed);
  double hoodAbsoluteEncoder = hoodAngler.getAbsoluteEncoder().getPosition();
  
  /**
   * Nothing done in constructor.
   */
  public HoodAngler() {}

  /**
   * Sets the power level of the hood angler motor to adjust the angle of the shooter hood.
   * @param power The power level to set the hood angler motor to, typically between -1.0 and 1.0.
   */
  public void adjust(double power) {
    // Needs limits
    // Needs PID
    if (Constants.OperatorConstants.DYNAMIC_POWER_CONTROL && power != 0) {
      power = SmartDashboard.getNumber("HoodAngle V", 0.1);
    }
    /* Tune limits
    if(power<=0){
      if(getPos() <= 0){ 
        hoodAngler.set(-power);
      }
      else{
        hoodAngler.set(0);
      }
    }
    else{
      if(getPos() > -20.7){
        hoodAngler.set(-power);
      }
      else{
        hoodAngler.set(0);
      }
    } */
    SmartDashboard.putNumber("ES AbsPos", getPos()); // Doesn't show up
    hoodAngler.set(-power);
  }

  public double getPos(){
    return hoodAbsoluteEncoder;
  }

  /**
   * Periodically updates the SmartDashboard with the current position of the hood angler.
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("HOOD Pos", hoodAbsoluteEncoder);
  }
}
