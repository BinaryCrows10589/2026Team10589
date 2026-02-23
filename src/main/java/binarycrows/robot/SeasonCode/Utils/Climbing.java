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

    //TODO: climb autopositoning is one-point
    // likely make these not be Auton objects but instead just CMStateRequests in a group...

    //TODO: Implement auto positioning and stuff in these
    public static void climbLeft() {
        Auton auton = new Auton(Pose2d.kZero, Climbing::buildLeft);
        auton.buildAuton();
        auton.dispatchSelf();
    }

    public static void climbRight() {
        Auton auton = new Auton(Pose2d.kZero, Climbing::buildRight);
        auton.buildAuton();
        auton.dispatchSelf();
    }

    public static void climbCenterLeft() {
        Auton auton = new Auton(Pose2d.kZero, Climbing::buildCenterLeft);
        auton.buildAuton();
        auton.dispatchSelf();
    }

    public static void climbCenterRight() {
        Auton auton = new Auton(Pose2d.kZero, Climbing::buildCenterRight);
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

    public static StateRequest[] buildCenterLeft() {
        return new StateRequest[] {
            new CMStateRequest(
                new CMTrajectory("climbCenterLeft", null, null, null, null, false, null, 0, 0)
            )
        };
    }

    public static StateRequest[] buildCenterRight() {
        return new StateRequest[] {
            new CMStateRequest(
                new CMTrajectory("climbCenterRight", null, null, null, null, false, null, 0, 0)
            )
        };
    }
}
