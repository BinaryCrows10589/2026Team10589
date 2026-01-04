package binarycrows.robot.SeasonCode.Constants;

import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;

public final class PoseEstimatorConstants {
    public static final Vector<N3> swerveDrivePoseEstimateTrust = VecBuilder.fill(0.05, 0.05, 0.1);
    public static final Vector<N3> visionPoseEstimateTrust = VecBuilder.fill(.2, .2, 0);
    public static final Vector<N3> questNavPoseEstimateTrust = VecBuilder.fill(.01, .01, 0.01);

    public static final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    public static final double maxAmbiguity = .02;
    public static final Pose2d maxPoseDeltaFromCurrent = new Pose2d(1, 1, Rotation2d.fromDegrees(10));
    public static OriginPosition originPosition = OriginPosition.kBlueAllianceWallRightSide;
    public static final int[] tagWhitelistRed = new int[] {};
    public static final int[] tagWhitelistBlue = new int[] {};
    // Index in first array is tag ID, index in second ID is blueX, blueY, blueRotDeg, redX, redY, redRotDeg
    public static final Map<String, double[][]> tagFudgeOffsets = Map.of(
      "FLModuleCam", new double[][] {
                        {0, 0, 0, 0, 0, 0}, // Tag 1
                        {0, 0, 0, 0, 0, 0}, // Tag 2
                        {0, 0, 0, 0, 0, 0}, // Tag 3
                        {0, 0, 0, 0, 0, 0}, // Tag 4
                        {0, 0, 0, 0, 0, 0}, // Tag 5
                        {0, 0, 0, 0, 0, 0}, // Tag 6
                        {0, 0, 0, 0, 0, 0}, // Tag 7
                        {0, 0, 0, 0, 0, 0}, // Tag 8
                        {0, 0, 0, 0, 0, 0}, // Tag 9
                        {0, 0, 0, 0, 0, 0}, // Tag 10
                        {0, 0, 0, 0, 0, 0}, // Tag 11
                        {0, 0, 0, 0, 0, 0}, // Tag 12
                        {0, 0, 0, 0, 0, 0}, // Tag 13
                        {0, 0, 0, 0, 0, 0}, // Tag 14
                        {0, 0, 0, 0, 0, 0}, // Tag 15
                        {0, 0, 0, 0, 0, 0}, // Tag 16
                        {0, 0, 0, 0, 0, 0}, // Tag 17
                        {0, 0, 0, 0, 0, 0}, // Tag 18
                        {0, 0, 0, 0, 0, 0}, // Tag 19
                        {0, 0, 0, 0, 0, 0}, // Tag 20
                        {0, 0, 0, 0, 0, 0}, // Tag 21
                        {0, 0, 0, 0, 0, 0}, // Tag 22
                        {0, 0, 0, 0, 0, 0}, // Tag 23
                        {0, 0, 0, 0, 0, 0}, // Tag 24
                        {0, 0, 0, 0, 0, 0}, // Tag 25
                        {0, 0, 0, 0, 0, 0}, // Tag 26
                        {0, 0, 0, 0, 0, 0}, // Tag 27
                        {0, 0, 0, 0, 0, 0}, // Tag 28
                        {0, 0, 0, 0, 0, 0}, // Tag 29
                        {0, 0, 0, 0, 0, 0}, // Tag 30
                        {0, 0, 0, 0, 0, 0}, // Tag 31
                        {0, 0, 0, 0, 0, 0}, // Tag 32
    },
  "BLModuleCam", new double[][] {
                        {0, 0, 0, 0, 0, 0}, // Tag 1
                        {0, 0, 0, 0, 0, 0}, // Tag 2
                        {0, 0, 0, 0, 0, 0}, // Tag 3
                        {0, 0, 0, 0, 0, 0}, // Tag 4
                        {0, 0, 0, 0, 0, 0}, // Tag 5
                        {0, 0, 0, 0, 0, 0}, // Tag 6
                        {0, 0, 0, 0, 0, 0}, // Tag 7
                        {0, 0, 0, 0, 0, 0}, // Tag 8
                        {0, 0, 0, 0, 0, 0}, // Tag 9
                        {0, 0, 0, 0, 0, 0}, // Tag 10
                        {0, 0, 0, 0, 0, 0}, // Tag 11
                        {0, 0, 0, 0, 0, 0}, // Tag 12
                        {0, 0, 0, 0, 0, 0}, // Tag 13
                        {0, 0, 0, 0, 0, 0}, // Tag 14
                        {0, 0, 0, 0, 0, 0}, // Tag 15
                        {0, 0, 0, 0, 0, 0}, // Tag 16
                        {0, 0, 0, 0, 0, 0}, // Tag 17
                        {0, 0, 0, 0, 0, 0}, // Tag 18
                        {0, 0, 0, 0, 0, 0}, // Tag 19
                        {0, 0, 0, 0, 0, 0}, // Tag 20
                        {0, 0, 0, 0, 0, 0}, // Tag 21
                        {0, 0, 0, 0, 0, 0}, // Tag 22
                        {0, 0, 0, 0, 0, 0}, // Tag 23
                        {0, 0, 0, 0, 0, 0}, // Tag 24
                        {0, 0, 0, 0, 0, 0}, // Tag 25
                        {0, 0, 0, 0, 0, 0}, // Tag 26
                        {0, 0, 0, 0, 0, 0}, // Tag 27
                        {0, 0, 0, 0, 0, 0}, // Tag 28
                        {0, 0, 0, 0, 0, 0}, // Tag 29
                        {0, 0, 0, 0, 0, 0}, // Tag 30
                        {0, 0, 0, 0, 0, 0}, // Tag 31
                        {0, 0, 0, 0, 0, 0}, // Tag 32
    });

    /*
    
    X: Positive -> Forward from robot center
    Y: Positive -> Left from robot center
    Z: Positive -> Up from robot center
    Yaw (Z): Rotation -> Counter-clockwise (right-handed) rotation around the Z axis
    Pitch (Y): Rotation -> Counter-clockwise (right-handed) rotation around the Y axis
    Roll (X): Rotation -> Counter-clockwise (right-handed) rotation around the X axis
 */
    public static final Transform3d robotToQuestOffset = new Transform3d(.34, 0, 0, Rotation3d.kZero);

    public static final Transform3d frontLeftCameraToCenter = new Transform3d(.37, .31, .904875, new Rotation3d(0, Rotation2d.fromDegrees(-15).getRadians(), Rotation2d.fromDegrees(45).getRadians()));
    public static final Transform3d frontRightCameraToCenter = new Transform3d(.37, -.31, .904875, new Rotation3d(0, Rotation2d.fromDegrees(-15).getRadians(), Rotation2d.fromDegrees(-45).getRadians()));
}
