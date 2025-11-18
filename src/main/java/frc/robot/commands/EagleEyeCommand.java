package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Globals;
import frc.robot.subsystems.EagleEye;

public class EagleEyeCommand extends Command {
  /** Creates a new EagleEyeCommand. */
  EagleEye eagleeye = new EagleEye();

  public EagleEyeCommand(frc.robot.Globals.EagleEye eagleeye2) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.eagleeye = eagleeye;
    addRequirements(eagleeye);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("inpath", Globals.inPath);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}