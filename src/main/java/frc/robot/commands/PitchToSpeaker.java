package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ShootingHelpers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Globals.EagleEye;
import frc.robot.subsystems.Indexer;

public class PitchToSpeaker extends Command {
  Indexer indexer;
  double shootAngle;
  double direction;
  double lastMeasurement = 0;
  public boolean found;

  public PitchToSpeaker(Indexer indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
    this.indexer = indexer;
    SmartDashboard.putNumber("Test Angle", Constants.SetPointConstants.ARM_SPEAKER);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!OperatorConstants.SHOOTING_DATA_COLLECTION_MODE){
      Translation2d speakerPos = ShootingHelpers.getTargetPos();

      double targetAngle = ShootingHelpers.angleInterp(speakerPos);

      indexer.angle(targetAngle);
    }else{
      SmartDashboard.putNumber("dist", ShootingHelpers.getTargetPos().getDistance(EagleEye.position.getTranslation()));
      indexer.angle(SmartDashboard.getNumber("Test Angle", Constants.SetPointConstants.ARM_SPEAKER));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
