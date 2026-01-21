// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ShootingHelpers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Globals.EagleEye;
import frc.robot.subsystems.Indexer;

/**
 * Command that pitches the indexer to face the speaker based on distance.
 */
public class PitchToSpeaker extends Command {
  private Indexer indexer;

  /**
   * Sets Indexer subsystem and puts test angle to SmartDashboard.
   * 
   * @param indexer
   */
  public PitchToSpeaker(Indexer indexer) {
    addRequirements(indexer);
    this.indexer = indexer;
    SmartDashboard.putNumber("Test Angle", Constants.SetPointConstants.ARM_SPEAKER);
  }

  /**
   * No initialization needed.
   */
  @Override
  public void initialize() {
  }

  /**
   * If in test mode, gets distance to target and sets indexer angle to test angle
   * from SmartDashboard.
   * If not in test mode, gets target position and sets indexer angle based on
   * interpolation continously.
   */
  @Override
  public void execute() {
    if (!OperatorConstants.SHOOTING_DATA_COLLECTION_MODE) {
      Translation2d speakerPos = ShootingHelpers.getTargetPos();

      double targetAngle = ShootingHelpers.angleInterp(speakerPos);

      indexer.angle(targetAngle);
    } else {
      SmartDashboard.putNumber("dist", ShootingHelpers.getTargetPos().getDistance(EagleEye.position.getTranslation()));
      indexer.angle(SmartDashboard.getNumber("Test Angle", Constants.SetPointConstants.ARM_SPEAKER));
    }
  }

  /**
   * No action needed on end.
   */
  @Override
  public void end(boolean interrupted) {
  }

  /**
   * Never finishes on its own.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}