// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToPointCommand extends Command {
  /** Creates a new DriveToPointCommand. */
  TargetPoints point;

  public DriveToPointCommand(TargetPoints point) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.point = point;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Command pathfindingCommand = AutoBuilder.pathfindToPose(point.get(), new PathConstraints(3, 3, 2*Math.PI, 4*Math.PI), 0);
    Field2d field = new Field2d();
    field.setRobotPose(point.get());
    SmartDashboard.putData("target point", field);
    pathfindingCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
