package binarycrows.robot.SeasonCode.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

public final class VisionConstants {
    public static final Vector<N3> swerveDrivePoseEstimateTrust = VecBuilder.fill(0.05, 0.05, 0.1);
    public static final Vector<N3> visionPoseEstimateTrust = VecBuilder.fill(.2, .2, 0);

    public static final AprilTagFieldLayout aprilTagLayout = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField();
    public static OriginPosition originPosition = OriginPosition.kBlueAllianceWallRightSide;
}
