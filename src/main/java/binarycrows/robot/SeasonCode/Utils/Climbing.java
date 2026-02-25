package binarycrows.robot.SeasonCode.Utils;

import binarycrows.robot.StateRequest;
import binarycrows.robot.CrowMotion.UserSide.CMAutonPoint;
import binarycrows.robot.CrowMotion.UserSide.CMEvent;
import binarycrows.robot.CrowMotion.UserSide.CMRotation;
import binarycrows.robot.CrowMotion.UserSide.CMStateRequest;
import binarycrows.robot.CrowMotion.UserSide.CMTrajectory;
import binarycrows.robot.CrowMotion.UserSide.CMTrajectory.TrajectoryPriority;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.SeasonCode.Constants.ControlConstants;
import binarycrows.robot.SeasonCode.Constants.FieldConstants;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Climber.ClimberStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveStateRequest;
import binarycrows.robot.StateRequestGroup.SequentialGroup;
import binarycrows.robot.Utils.Auton.Auton;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Climbing {

    public static void climbLeft() {
        new SequentialGroup(StateRequestPriority.NORMAL, 15*1000, buildLeft()).dispatchSelf();
    }

    public static void climbRight() {
        new SequentialGroup(StateRequestPriority.NORMAL, 15*1000, buildRight()).dispatchSelf();
    }

    public static void climbCenterLeft() {
        new SequentialGroup(StateRequestPriority.NORMAL, 15*1000, buildCenterLeft()).dispatchSelf();
    }

    public static void climbCenterRight() {
        new SequentialGroup(StateRequestPriority.NORMAL, 15*1000, buildCenterRight()).dispatchSelf();
    }

    public static void cancelClimb() {
        (new StateRequest<>(DriveStateRequest.TELEOP_DRIVE, StateRequestPriority.NORMAL)).dispatchSelf();
    }

    public static StateRequest[] buildLeft() {
        return build("Left", new CMAutonPoint(1, 4.881, 0, 0, 0, 0), new CMAutonPoint(1, 4.7, 0, 0, 0, 0), 0);
    }

    public static StateRequest[] buildRight() {
        return build("Right", new CMAutonPoint(1, 2.265, 0, 0, 0, 0), new CMAutonPoint(1, 2.8, 0, 0, 0, 0), 180);
    }

    public static StateRequest[] buildCenterLeft() {
        return build("CenterLeft", new CMAutonPoint(1.614, 3.745, 0, 0, 0, 0), new CMAutonPoint(1.443, 3.745, 0, 0, 0, 0), 90);
    }

    public static StateRequest[] buildCenterRight() {
        return build("CenterRight", new CMAutonPoint(1.614, 3.745, 0, 0, 0, 0), new CMAutonPoint(1.443, 3.745, 0, 0, 0, 0), -90);
    }

    private static StateRequest[] build(String side, CMAutonPoint startPoint, CMAutonPoint endPoint, double lineUpRotation) {
        return new StateRequest[] {
            (MetaConstants.isReal) ? new StateRequest<>(ClimberStateRequest.UP, StateRequestPriority.NORMAL) : null,

            new CMStateRequest(
                new CMTrajectory("climb" + side + "LineUp", 
                new CMAutonPoint[] {
                    startPoint
                }, 
                new CMRotation[] {
                    new CMRotation(lineUpRotation, 0, 1, 1)
                }, 
                new CMEvent[] {}, 
                TrajectoryPriority.SPLIT_PROPORTIONALLY, 
                true, 
                new double[] {0.01, 0.01}, 
                0.04, 
                5
                )
            ),

            new CMStateRequest(
                new CMTrajectory("climb" + side + "Hook", 
                new CMAutonPoint[] {
                    endPoint
                }, 
                new CMRotation[] {
                    new CMRotation(lineUpRotation, 0, 1, 1)
                }, 
                new CMEvent[] {}, 
                TrajectoryPriority.SPLIT_PROPORTIONALLY, 
                true, 
                new double[] {0.01, 0.01}, 
                0.04, 
                5
                )
            ),

            (MetaConstants.isReal) ? new StateRequest<>(ClimberStateRequest.DOWN, StateRequestPriority.NORMAL) : null
        };
    }
}
