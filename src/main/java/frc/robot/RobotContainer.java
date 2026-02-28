// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimbOnlyCommand;
import frc.robot.commands.EagleEyeCommand;
import frc.robot.commands.FeedRollOnly;
import frc.robot.commands.HoodAngleOnlyCommand;
import frc.robot.commands.IntakeOnlyCommand;
import frc.robot.commands.PositionIntakeCommand;
import frc.robot.commands.ShootingOnlyCommand;
import frc.robot.commands.SpindexOnlyCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.EagleEye;
import frc.robot.subsystems.FeedRoller;
import frc.robot.subsystems.HoodAngler;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
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
  private final HoodAngler hoodAngler = new HoodAngler();
  private final Intake intake =  new Intake();
  private final Shooter shooter = new Shooter();
  private final Climber climber = new Climber();
  private final Spindexer spindexer = new Spindexer();
  private final FeedRoller feedRoller = new FeedRoller();

  // Auto Chooser
  private final SendableChooser<Command> auto;

  // Basic Teleop Commands
  private final HoodAngleOnlyCommand hoodAngleOnlyCommandUp = new HoodAngleOnlyCommand(hoodAngler, 0.1);
  private final HoodAngleOnlyCommand hoodAngleOnlyCommandDown = new HoodAngleOnlyCommand(hoodAngler, -0.1);
  private final IntakeOnlyCommand intakeOnlyCommand = new IntakeOnlyCommand(intake, 0.1);
  private final IntakeOnlyCommand outtakeOnlyCommand = new IntakeOnlyCommand(intake, -0.1);
  private final ShootingOnlyCommand shootingOnlyCommand = new ShootingOnlyCommand(shooter, 0.1);
  private final ClimbOnlyCommand climbOnlyCommandUp = new ClimbOnlyCommand(climber, 0.1);
  private final ClimbOnlyCommand climbOnlyCommandDown = new ClimbOnlyCommand(climber, -0.1);
  private final SpindexOnlyCommand spindexOnlyCommand = new SpindexOnlyCommand(spindexer, 0.1);
  private final FeedRollOnly feedRollOnly = new FeedRollOnly(feedRoller, 0.1);
  private final PositionIntakeCommand deployIntakeUp = new PositionIntakeCommand(intake, -0.1);
  private final PositionIntakeCommand deployIntakeDown = new PositionIntakeCommand(intake, 0.1);

  // Shoot Chain Teleop Commands
  private final FeedRollOnly intakeChainFeedRollCommand2 = new FeedRollOnly(feedRoller, 0.1);
  private final SpindexOnlyCommand intakeChainSpindexCommand1 = new SpindexOnlyCommand(spindexer, 0.1);
  private final ShootingOnlyCommand intakeChainShootingCommand3 = new ShootingOnlyCommand(shooter, 0.1);


 
  // Controllers
  private XboxController buttonsXbox;
  private XboxController driverXbox;
  private Joystick rightJoystick;
  private Joystick leftJoystick;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    SmartDashboard.putNumber("HoodAngle V", 0.1);
    SmartDashboard.putNumber("Intake V", 0.1);
    SmartDashboard.putNumber("IntakeDeploy V", 0.1);
    SmartDashboard.putNumber("Shooting V", 0.1);
    SmartDashboard.putNumber("Climber V", 0.1);
    SmartDashboard.putNumber("Spindexer V", 0.1);
    SmartDashboard.putNumber("Feed Roll V", 0.1);

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
    //driveBase.setMotorBrake(true);
    
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
          () -> MathUtil.applyDeadband(-leftJoystick.getRawAxis(1) * Globals.inversion, OperatorConstants.LEFT_Y_DEADBAND),
          () -> MathUtil.applyDeadband(-leftJoystick.getRawAxis(0) * Globals.inversion, OperatorConstants.LEFT_X_DEADBAND),
          () -> -rightJoystick.getRawAxis(0));
    } 

    // Map controller buttons
    configureBindings();

    // Set default commands
    driveBase.setDefaultCommand(driveCommand);
    if (Constants.EagleEyeConstants.EAGLEEYE_ENABLED)
    {
      eagleEye.setDefaultCommand(eagleEyeCommand);
    }

    auto = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("chooseAuto", auto);

    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  private void configureBindings() {

    if (OperatorConstants.XBOX_DRIVE) {
      // Xbox Controller Bindings
    } else {
      new JoystickButton(leftJoystick, 4).onTrue((new InstantCommand(driveBase::zeroGyro)));

      new JoystickButton(buttonsXbox, 1).whileTrue(intakeOnlyCommand); // A

      //new JoystickButton(buttonsXbox, 4).whileTrue(deployIntake); // Y

      new POVButton(buttonsXbox, 0).whileTrue(climbOnlyCommandUp); // DPad Up
      new POVButton(buttonsXbox, 180).whileTrue(climbOnlyCommandDown); // DPad Down

      new POVButton(buttonsXbox, 90).whileTrue(hoodAngleOnlyCommandUp); // DPad Right
      new POVButton(buttonsXbox, 270).whileTrue(hoodAngleOnlyCommandDown); // DPad Left

      new JoystickButton(buttonsXbox, 3).whileTrue(spindexOnlyCommand); // X
      //new JoystickButton(buttonsXbox, 4).whileTrue(feedRollOnly); // Y

      new JoystickButton(buttonsXbox, 8).whileTrue(shootingOnlyCommand); // Right Stick Button
      new JoystickButton(buttonsXbox, 8).whileTrue(intakeChainSpindexCommand1.withTimeout(0.7)
      .andThen(new ParallelCommandGroup(
            intakeChainSpindexCommand1,
            intakeChainFeedRollCommand2,
            intakeChainShootingCommand3
        ))); // A


      //new JoystickButton(leftJoystick, 3).onTrue(new DriveToPointCommand(TargetPoints.TAG_28, "Forward"));
      //new JoystickButton(leftJoystick, 3).whileTrue(new FaceTargetCommand(driveBase, TargetPoints.TAG_25.get()));
      //new JoystickButton(buttonsXbox, 4).whileTrue(new RotateToAngleExtended(driveBase, TargetPoints.TAG_25.get()));
      
     // new Trigger(buttonsXbox.rightTrigger(0.5, null)).whileTrue(shootingOnlyCommand); // Right Trigger
    }
  }

  public Command getAutonomousCommand() {
    return auto.getSelected();
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