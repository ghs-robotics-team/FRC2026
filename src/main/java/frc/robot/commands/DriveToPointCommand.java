// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drives to a specified point on the field using pathfinding.
 */
public class DriveToPointCommand extends Command {
  private TargetPoints point;

  /**
   * Sets the point for this command to drive to.
   * 
   * @param point The point to drive to.
   */
  public DriveToPointCommand(TargetPoints point) {
    this.point = point;
  }

  /**
   * Creates a pathfinding command to drive to the specified point
   * using a given field and AutoBuilder.
   */
  @Override
  public void initialize() {
    Command pathfindingCommand = AutoBuilder.pathfindToPose(point.get(),
        new PathConstraints(3, 3, 2 * Math.PI, 4 * Math.PI), 0);
    Field2d field = new Field2d();
    field.setRobotPose(point.get());
    SmartDashboard.putData("target point", field);
    pathfindingCommand.schedule();
  }

  /**
   * No need to execute anything here since the pathfinding command is scheduled
   * in initialize().
   */
  @Override
  public void execute() {
  }

  /**
   * No need do anything when the command ends.
   */
  @Override
  public void end(boolean interrupted) {
  }

  /**
   * This command finishes immediately after scheduling the pathfinding command.
   */
  @Override
  public boolean isFinished() {
    return true;
  }
}
