// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveToPointCommand;
import frc.robot.commands.EagleEyeCommand;
import frc.robot.commands.FaceTargetCommand;
import frc.robot.commands.RotateToAngleExtended;
import frc.robot.commands.TargetPoints;
import frc.robot.subsystems.EagleEye;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final SwerveSubsystem driveBase;
  private final EagleEye eagleEye;
  private final EagleEyeCommand eagleEyeCommand;

  // Teleop Commands

  // Controllers
  private XboxController buttonsXbox;
  private XboxController driverXbox;
  private Joystick rightJoystick;
  private Joystick leftJoystick;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    if (Constants.EagleEyeConstants.EAGLEEYE_ENABLED) {
      eagleEye = new EagleEye();
      eagleEyeCommand = new EagleEyeCommand(eagleEye);
    } else {
      eagleEye = null;
      eagleEyeCommand = null;
    }
    
    SmartDashboard.putNumber("SS Workshop MaxSpeed", Constants.OperatorConstants.WORKSHOP_MAX_SPEED);
    SmartDashboard.putBoolean("SS Workshop Mode On", Constants.OperatorConstants.WORKSHOP_MODE);
    SmartDashboard.putBoolean("Record Data", false);
    SmartDashboard.putBoolean("Record Time Data", false);

    driveBase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), Constants.DrivebaseConstants.YAGSL_CONFIG_FOLDER));
    driveBase.setMotorBrake(true);
    
    // Connect the controllers
    if (Constants.OperatorConstants.XBOX_DRIVE) {
      driverXbox = new XboxController(0);
      if (DriverStation.isJoystickConnected(1)) {
        buttonsXbox = new XboxController(1);
      } else {
        buttonsXbox = driverXbox;
      }
      Globals.buttonsXbox = buttonsXbox;
    } else {
      rightJoystick = new Joystick(0);
      leftJoystick = new Joystick(1);
      buttonsXbox = new XboxController(2);
      Globals.buttonsXbox = buttonsXbox;
    }

    // Configure DriveCommand
    Command driveCommand = null;
    if (OperatorConstants.XBOX_DRIVE) {
      driveCommand = driveBase.driveCommand(
          () -> MathUtil.applyDeadband(-driverXbox.getLeftY() * Globals.inversion, OperatorConstants.LEFT_Y_DEADBAND),
          () -> MathUtil.applyDeadband(-driverXbox.getLeftX() * Globals.inversion, OperatorConstants.LEFT_X_DEADBAND),
          () -> MathUtil.applyDeadband(-driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND));
    } else {
      driveCommand = driveBase.driveCommand(
          () -> MathUtil.applyDeadband(leftJoystick.getRawAxis(1) * Globals.inversion,
              OperatorConstants.LEFT_Y_DEADBAND),
          () -> MathUtil.applyDeadband(leftJoystick.getRawAxis(0) * Globals.inversion,
              OperatorConstants.LEFT_X_DEADBAND),
          () -> rightJoystick.getRawAxis(0) * Globals.inversion);
    }

    // Map controller buttons
    configureBindings();

    // Set default commands
    driveBase.setDefaultCommand(driveCommand);
    if (Constants.EagleEyeConstants.EAGLEEYE_ENABLED)
    {
      eagleEye.setDefaultCommand(eagleEyeCommand);
    }

    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  private void configureBindings() {

    if (OperatorConstants.XBOX_DRIVE) {
      /*
       * Driver Xbox bindings
       * +-----------------------------+-------------------------------+
       * | Control                     | Action                        |
       * +-----------------------------+-------------------------------+
       * | Start                       | Zero gyro                     |
       * | B (2)                       | Drive to TAG_28 (forward)     |
       * | Y (4)                       | Face Tag                      |
       * +-----------------------------+-------------------------------+
       */
      new JoystickButton(driverXbox, 8).onTrue((new InstantCommand(driveBase::zeroGyro)));
      new JoystickButton(driverXbox, 2).onTrue(new DriveToPointCommand(TargetPoints.TAG_28, "Forward"));
      new JoystickButton(driverXbox, 4).whileTrue(new FaceTargetCommand(driveBase, TargetPoints.TAG_25.get()));
    } else {
      /*
       * Joystick bindings
       * +------------------------------+-------------------------------+
       * | Control                      | Action                        |
       * +------------------------------+-------------------------------+
       * | Left Joystick Button 4       | Zero gyro                     |
       * | Left Joystick Button 3       | Drive to TAG_28 (forward)     |
       * | Y (4)                        | Face Tag                      |
       * +------------------------------+-------------------------------+
       */
      new JoystickButton(leftJoystick, 4).onTrue((new InstantCommand(driveBase::zeroGyro)));
      new JoystickButton(leftJoystick, 3).onTrue(new DriveToPointCommand(TargetPoints.TAG_28, "Forward"));
      //new JoystickButton(leftJoystick, 3).whileTrue(new FaceTargetCommand(driveBase, TargetPoints.TAG_25.get()));
      new JoystickButton(buttonsXbox, 4).whileTrue(new RotateToAngleExtended(driveBase, TargetPoints.TAG_25.get()));
    }
  }

  public Command getAutonomousCommand() {
    return null;
  }

  /**
   * Sets the motor brake mode for the drivebase.
   * 
   * @param brake
   */
  public void setMotorBrake(boolean brake) {
    driveBase.setMotorBrake(brake);
  }
}