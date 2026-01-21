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

/**
 * Shooter subsystem for controlling the shooter motors.
 */
public class Shooter extends SubsystemBase {
  private TalonFX BottomMotor = new TalonFX(17);
  private TalonFX TopMotor = new TalonFX(18);
  private double lastReadTimestamp = 0;

  /**
   * Nothing done in init.
   */
  public Shooter() {
  }

  /**
   * Spins the shooter motors at the specified speed.
   * (Negative for bottom motor, positive for top motor)
   * 
   * @param speed Speed from -1 to 1
   * @param isAmp Whether to use the AMP shooting voltages
   */
  public void spin(double speed, boolean isAmp) {
    if (isAmp) {
      BottomMotor.setVoltage(-speed * 12);
      TopMotor.setVoltage(speed * 9);
    } else {
      BottomMotor.setVoltage(-speed * 12);
      TopMotor.setVoltage(speed * 12);
    }
  }

  /**
   * Spins the shooter motors at the specified speed.
   * (Negative for bottom motor, positive for top motor)
   * 
   * @param speed Speed from -1 to 1
   */
  public void spin(double speed) {
    spin(speed, false);
  }

  /**
   * Checks if either shooter motor is above maximum temperature.
   * 
   * @return True if above maximum temperature, false otherwise.
   */
  public boolean getAboveMaxTemp() {
    return Math.max(LastTempMeasurement.topShooterTemp,
        LastTempMeasurement.bottomShooterTemp) >= MaximumTemps.MaxFalconTemp;
  }

  /**
   * Updates the shooter motor temperatures on the SmartDashboard.
   */
  @Override
  public void periodic() {
    if (Timer.getFPGATimestamp() - lastReadTimestamp > 1) {
      lastReadTimestamp = Timer.getFPGATimestamp();
      LastTempMeasurement.topShooterTemp = TopMotor.getDeviceTemp().getValueAsDouble();
      LastTempMeasurement.bottomShooterTemp = BottomMotor.getDeviceTemp().getValueAsDouble();
    }
    SmartDashboard.putNumber("Top Motor Temp", LastTempMeasurement.topShooterTemp);
    SmartDashboard.putNumber("Bottom Motor Temp", LastTempMeasurement.bottomShooterTemp);
  }
}