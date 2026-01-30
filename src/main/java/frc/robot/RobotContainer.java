// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.EagleEye;
import frc.robot.commands.EagleEyeCommand;
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
  private final EagleEye eagleeye = new EagleEye();
  private final EagleEyeCommand eagleEyeCommand = new EagleEyeCommand(eagleeye);

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

    SmartDashboard.putNumber("Workshop MaxSpeed", Globals.workShopSettings.maxSpeed);
    SmartDashboard.putBoolean("Workshop Mode On", Constants.OperatorConstants.WORKSHOP_MODE);
    SmartDashboard.putBoolean("Record Data", false);
    SmartDashboard.putBoolean("Record Time Data", false);

    if (Globals.robotSwerveConfig.equals("8702")){
          driveBase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve8702"));
    } else {
          driveBase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    }
    //((Subsystem) eagleeye).setDefaultCommand(eagleeyecommand);

    // Configure the trigger bindings

    if (Constants.OperatorConstants.XBOX_DRIVE) {
      driverXbox = new XboxController(0);
      buttonsXbox = new XboxController(1);
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
          () -> 0.0, //MathUtil.applyDeadband(leftjoystick.getRawAxis(0) * Globals.inversion,
              //OperatorConstants.LEFT_X_DEADBAND),
          () -> 0.0); //-rightjoystick.getRawAxis(0));
    }

    configureBindings();
    driveBase.setDefaultCommand(driveCommand);
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * @return the command to run in autonomous
   */
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