package binarycrows.robot.SeasonCode.Utils;

import binarycrows.robot.StateRequest;
import binarycrows.robot.CrowMotion.UserSide.CMStateRequest;
import binarycrows.robot.CrowMotion.UserSide.CMTrajectory;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.SeasonCode.Constants.ControlConstants;
import binarycrows.robot.SeasonCode.Constants.FieldConstants;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveStateRequest;
import binarycrows.robot.Utils.Auton.Auton;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Climbing {

    //TODO: Ask Eli if this is how it should work
    public static final Pose2d climbingStartingPositionBlue = new Pose2d(3.75, 2.0, Rotation2d.fromDegrees(180));
    public static final Pose2d climbingStartingPositionRed = new Pose2d(FieldConstants.fieldLengthMeters - 3.75, FieldConstants.fieldWidthMeters - 2.0, Rotation2d.fromDegrees(0));

    public static Pose2d getStartingPosition() {
        return MetaConstants.isBlueAlliance ? climbingStartingPositionBlue : climbingStartingPositionRed;
    }

    //TODO: Implement auto positioning and stuff in these
    public static void climbLeft() {
        Auton auton = new Auton(getStartingPosition(), Climbing::buildLeft);
        auton.buildAuton();
        auton.dispatchSelf();
    }

    public static void climbRight() {
        Auton auton = new Auton(getStartingPosition(), Climbing::buildRight);
        auton.buildAuton();
        auton.dispatchSelf();
    }

    public static void climbCenter() {
        Auton auton = new Auton(getStartingPosition(), Climbing::buildCenter);
        auton.buildAuton();
        auton.dispatchSelf();
    }

    public static void cancelClimb() {
        (new StateRequest<>(DriveStateRequest.TELEOP_DRIVE, StateRequestPriority.NORMAL)).dispatchSelf();
    }

    // TODO: Fill in control points
    public static StateRequest[] buildLeft() {
        return new StateRequest[] {
            new CMStateRequest(
                new CMTrajectory("climbLeft", null, null, null, null, false, null, 0, 0)
            )
        };
    }

    public static StateRequest[] buildRight() {
        return new StateRequest[] {
            new CMStateRequest(
                new CMTrajectory("climbRight", null, null, null, null, false, null, 0, 0)
            )
        };
    }

    public static StateRequest[] buildCenter() {
        return new StateRequest[] {
            new CMStateRequest(
                new CMTrajectory("climbCenter", null, null, null, null, false, null, 0, 0)
            )
        };
    }
}
