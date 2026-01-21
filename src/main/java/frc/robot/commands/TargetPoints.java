package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Enum containing important target points on the field with methods to get
 * the correct pose based on alliance color.
 */
public enum TargetPoints {
    STAGE_AMP(new Pose2d(4.171, 4.853, Rotation2d.fromDegrees(-52.520))),
    STAGE_SOURCE(new Pose2d(4.171, 3.325, Rotation2d.fromDegrees(52.520))),
    STAGE_CENTER(new Pose2d(5.83, 4.023, Rotation2d.fromDegrees(180))),
    AMP(new Pose2d(1.8495381307142784, 7.667825683964218, Rotation2d.fromDegrees(90))),
    SOURCE(new Pose2d(14.084, 1.003, Rotation2d.fromDegrees(-19))),
    SPEAKER_MID(new Pose2d(3.161, 5.587, Rotation2d.fromDegrees(179.5))),
    SPEAKER_LAUNCH(new Pose2d(2.526, 4.246, Rotation2d.fromDegrees(155.020))),
    SPEAKER_AMP(new Pose2d(2.643, 6.944, Rotation2d.fromDegrees(-156.584)));

    public Pose2d pose;

    /**
     * Sets the pose for the target point.
     * 
     * @param pose The Pose2d of the target point.
     */
    private TargetPoints(Pose2d pose) {
        this.pose = pose;
    }

    /**
     * Gets the pose adjusted for alliance color.
     * 
     * @return The adjusted Pose2d.
     */
    public Pose2d get() {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            return new Pose2d(16.5354 - pose.getX(), pose.getY(),
                    Rotation2d.fromDegrees(180).minus(pose.getRotation()));
        } else {
            return pose;
        }
    }
}