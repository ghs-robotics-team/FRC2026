// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Indexer subsystem for controlling the angle and shooting mechanism.
 */
public class Indexer extends SubsystemBase {
  private TalonFX Angler = new TalonFX(5);
  private SparkMax Driver = new SparkMax(16, MotorType.kBrushless);
  private PIDController pid;
  private double currentLocation;
  private double direction;
  private double setAngle = Constants.SetPointConstants.ARM_INTAKE;
  private double home = 0;
  
  /**
   * Initializes motor configurations and PID controller.
   */
  public Indexer() {
    this.pid = new PIDController(0.35, 0, 0.0005);// Old: 0.28, 0, 0.0005
    SmartDashboard.putNumber("Indexer-P", 0.35);
    SmartDashboard.putNumber("Indexer-I", 0);
    SmartDashboard.putNumber("Indexer-D", 0.0005);
    SmartDashboard.putNumber("Arm Speed", 6.5);
    home = Angler.getRotorPosition().getValueAsDouble();
  }

  /**
   * Sets the desired angle for the indexer.
   * 
   * @param angle Target angle.
   */
  public void angle(double angle) {
    setAngle = angle;
  }

  /**
   * Lowers the home position by one unit.
   */
  public void lowerHome() {
    home--;
  }

  /**
   * Sets the current position as the home position.
   */
  public void setHome() {
    home = Angler.getRotorPosition().getValueAsDouble();
  }

  /**
   * Activates the shooting mechanism at the specified speed.
   * 
   * @param dspeed Speed multiplier (0 to 1).
   */
  public void shoot(double dspeed) {
    Driver.setVoltage(dspeed * 12);
  }

  /**
   * Sets the driver motor to coast mode.
   */
  public void coast() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);
    Driver.configure(config, com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  /**
   * Sets the driver motor to brake mode.
   */
  public void brake() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    Driver.configure(config, com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
  }

  /**
   * Gets the current target angle.
   * 
   * @return Target angle.
   */
  public double getAngle() {
    return setAngle;
  }

  /**
   * Periodic method to update indexer position and PID control.
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Angle", Angler.getRotorPosition().getValueAsDouble());
    SmartDashboard.putNumber("SetAngle", setAngle);

    double P = SmartDashboard.getNumber("Indexer-P", 0.28);
    double I = SmartDashboard.getNumber("Indexer-I", 0.0);
    double D = SmartDashboard.getNumber("Indexer-D", 0.0005);
    double Speed = SmartDashboard.getNumber("Arm Speed", 6.5);

    // Set PID numbers
    pid.setP(P);
    pid.setI(I);
    pid.setD(D);

    currentLocation = Angler.getRotorPosition().getValueAsDouble() - home;
    SmartDashboard.putNumber("CurrentLocation", currentLocation);
    direction = pid.calculate(currentLocation, setAngle);
    direction = MathUtil.clamp(direction, -Speed, Speed);
    SmartDashboard.putNumber("Old direction", direction);

    if (pid.getPositionError() > -0.25 && pid.getPositionError() < 0.25) {
      // deadzone
      Angler.setVoltage(0);
    } else {
      Angler.setVoltage(direction);
    }
  }
}