package binarycrows.robot.SeasonCode.Constants;

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
    public static final int[] kExcludedTags = new int[] {};
    /* TODO: BOYNE, you may want to change the implimentaiton of this. 
      This was just a simply way I thought of first
      The system must give blue and red offsets for each tag for both x and y */
    // Index in first array is tag ID, index in second ID is blueX, blueY, blueRotDeg, redX, redY, redRotDeg
    public static final double[][] tagFudgeOffsets = new double[][] {
        {0, 0, 0, 0, 0, 0}, // Tag 1
        {0, 0, 0, 0, 0, 0}, // Tag 2
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 16
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
        {0, 0, 0, 0, 0, 0}, // Tag 3
    };

    /*
    
    X: Positive -> Forward from robot center
    Y: Positive -> Left from robot center
    Z: Positive -> Up from robot center
    Yaw (Z): Rotation -> Counter-clockwise (right-handed) rotation around the Z axis
    Pitch (Y): Rotation -> Counter-clockwise (right-handed) rotation around the Y axis
    Roll (X): Rotation -> Counter-clockwise (right-handed) rotation around the X axis
 */
    public static final Transform3d robotToQuestOffset = new Transform3d(.34, 0, 0, Rotation3d.kZero); //TODO: update to real quest position
}
