package binarycrows.robot.Utils;

import edu.wpi.first.math.geometry.Pose2d;

public class TolorenceUtil {
    public static boolean inPoseTolorence(Pose2d pose1, Pose2d pose2, Pose2d tolerance) {
    return Math.abs(pose1.getX() - pose2.getX()) <= tolerance.getX()
        && Math.abs(pose1.getY() - pose2.getY()) <= tolerance.getY()
        && Math.abs(
                pose1.getRotation().minus(pose2.getRotation()).getDegrees()
           ) <= tolerance.getRotation().getDegrees();
}

}
