// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MaximumTemps;
import frc.robot.Globals.LastTempMeasurement;

public class Shooter extends SubsystemBase {

  TalonFX BottomMotor = new TalonFX(17);
  TalonFX TopMotor = new TalonFX(18);
  double lastReadTopTemp = 0;
  double lastReadBottomTemp = 0;
  double lastReadTimestamp = 0;

  public Shooter() {
  }

  public void spin(double speed, boolean isAmp) {
    if(isAmp){
      BottomMotor.setVoltage(-speed*12);
      TopMotor.setVoltage(speed*9);
    }else{
      BottomMotor.setVoltage(-speed*12);
      TopMotor.setVoltage(speed*12);
    }
  }

  public void spin(double speed) {
    spin(speed, false);
  }

  public boolean getAboveMaxTemp(){
    return Math.max(LastTempMeasurement.topShooterTemp, LastTempMeasurement.bottomShooterTemp) >= MaximumTemps.MaxFalconTemp;
  }
  
  @Override
  public void periodic() {
    if(Timer.getFPGATimestamp() - lastReadTimestamp > 1){
      lastReadTimestamp = Timer.getFPGATimestamp();
      LastTempMeasurement.topShooterTemp = TopMotor.getDeviceTemp().getValueAsDouble();
      LastTempMeasurement.bottomShooterTemp = BottomMotor.getDeviceTemp().getValueAsDouble();
    }
    SmartDashboard.putNumber("Top Motor Temp", LastTempMeasurement.topShooterTemp);
    SmartDashboard.putNumber("Bottom Motor Temp", LastTempMeasurement.bottomShooterTemp);
  }
}
