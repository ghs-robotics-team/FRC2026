package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Globals;

public class DriveToPointCommand extends Command {
  /** Creates a new DriveToPointCommand. */
  TargetPoints point;
  String heading;

  public DriveToPointCommand(TargetPoints point, String heading) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.point = point;
    this.heading = heading;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Build Command based on Point and Constraints.
    Command pathfindingCommand;
    if(heading.equals("Forward")){
      if(Constants.OperatorConstants.WORKSHOP_MODE){
          pathfindingCommand = AutoBuilder.pathfindToPose(point.getForward(), new PathConstraints(Globals.workShopSettings.maxSpeed, 2, 2*Math.PI, 4*Math.PI), 0.0);
      } else{
          pathfindingCommand = AutoBuilder.pathfindToPose(point.getForward(), new PathConstraints(Constants.MAX_SPEED, 2, 2*Math.PI, 4*Math.PI), 0.0);
      }

    }
    else{
      if(Constants.OperatorConstants.WORKSHOP_MODE){
        pathfindingCommand = AutoBuilder.pathfindToPose(point.get(), new PathConstraints(Globals.workShopSettings.maxSpeed, 2, 2*Math.PI, 4*Math.PI), 0.0);
      } else{
        pathfindingCommand = AutoBuilder.pathfindToPose(point.get(), new PathConstraints(Constants.MAX_SPEED, 2, 2*Math.PI, 4*Math.PI), 0.0);
      }
    }
    
    // Setup Variables for Field and Robotpose.
    Field2d field = new Field2d();
    field.setRobotPose(point.get());
    SmartDashboard.putData("DTP target point", field);
    
    // Sends data to Globals to indicate that the robot is currently in a command.
    Globals.inPath = true;
    pathfindingCommand.andThen(new InstantCommand(() -> {
        Globals.inPath = false;
      })).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}