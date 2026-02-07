package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public enum TargetPoints {
    TAG_28(new Pose2d(Units.inchesToMeters(162.64), Units.inchesToMeters(25.37), Rotation2d.fromDegrees(0)));
    
    public Pose2d pose;

    private TargetPoints(Pose2d pose) {
        this.pose = pose;
    }

    public Pose2d distanceFromTag(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        x += Units.inchesToMeters(13.77) * Math.cos(pose.getRotation().getRadians()); 
        y += Units.inchesToMeters(13.77) * Math.sin(pose.getRotation().getRadians());
        return new Pose2d(x, y, Rotation2d.fromDegrees(90).plus(pose.getRotation()));
    }

    public static Pose2d tagPos(Pose2d pose, double inches){ 
        // Current X and Y Position of the Robot.
        double x = pose.getX();
        double y = pose.getY();
        double angle_degree;

        // Correcting for Weird WPILib Angle Measurements.
        if (inches >= 0) {
        angle_degree = pose.getRotation().getDegrees() - 90;
        } else {
        angle_degree = pose.getRotation().getDegrees() + 90;
        }
        double angle = Units.degreesToRadians(angle_degree + 90);

        // Calculate new X position based on Trigonometry
        if (inches >= 0) {
            x += Units.inchesToMeters(inches) * Math.cos(angle);
            } else {
            x -= Units.inchesToMeters(inches) * Math.cos(angle);
        }
        
        // Calculate new Y based on if the robot is moving right or left.
        if (inches >= 0) {
        y += Units.inchesToMeters(inches) * Math.sin(angle);
        } else {
        y -= Units.inchesToMeters(inches) * Math.sin(angle);
        }

        return new Pose2d(x, y, pose.getRotation());
    }

    public Pose2d get() {
        Pose2d newpose = distanceFromTag(pose);
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (alliance == Alliance.Red) {
            return new Pose2d(newpose.getX() + 8.57, newpose.getY(), newpose.getRotation());
        } else {
            return newpose;
        }
    }

    public Pose2d getForward() {
        Pose2d newpose = distanceFromTag(pose);
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (alliance == Alliance.Red) {
            return new Pose2d(newpose.getX() + 8.57, newpose.getY(), newpose.getRotation().plus(Rotation2d.fromDegrees(90)));
        } else {
            return new Pose2d(newpose.getX(), newpose.getY(), newpose.getRotation().plus(Rotation2d.fromDegrees(90)));
        }
    }

    public static void printPlaces(){
        for(TargetPoints point: TargetPoints.values()){
            System.out.println(point.name() + " :" + TargetPoints.tagPos(new Pose2d(point.get().getX(), point.get().getY(), point.get().getRotation()), -15)); //-2 or -15
        }
    }
}