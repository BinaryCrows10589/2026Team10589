package binarycrows.robot.SeasonCode.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {

    public static final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    public static final Transform3d frontLeftCameraToCenter = new Transform3d(.37, .31, .904875, new Rotation3d(0, Rotation2d.fromDegrees(-15).getRadians(), Rotation2d.fromDegrees(45).getRadians()));
    public static final Transform3d frontRightCameraToCenter = new Transform3d(.37, -.31, .904875, new Rotation3d(0, Rotation2d.fromDegrees(-15).getRadians(), Rotation2d.fromDegrees(-45).getRadians()));
}
//35.625

//.29m