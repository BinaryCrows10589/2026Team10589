package binarycrows.robot.SeasonCode.Autons;

import binarycrows.robot.StateRequest;
import binarycrows.robot.CrowMotion.UserSide.CMEvent;
import binarycrows.robot.CrowMotion.UserSide.CMRotation;
import binarycrows.robot.CrowMotion.UserSide.CMStateRequest;
import binarycrows.robot.CrowMotion.UserSide.CMTrajectory;
import binarycrows.robot.CrowMotion.UserSide.CMTrajectory.TrajectoryPriority;
import binarycrows.robot.Enums.StateRequestGroupChildTimeoutBehavior;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.SeasonCode.Autons.Data.Paths;
import binarycrows.robot.SeasonCode.Autons.Data.Points;
import binarycrows.robot.SeasonCode.SubStateManagers.Shooting.ShootingStateRequest;
import binarycrows.robot.StateRequestGroup.SequentialGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class HumanPlayerTrench_Wall_Shoot_L_Shoot_P_Shoot {
    public static Pose2d startingPoint = new Pose2d(Points.startPosition_HumanPlayerTrench_Wall.geTranslation2d(), Rotation2d.fromDegrees(-90));

    public static SequentialGroup getAutonomous() {
        return new SequentialGroup(
            StateRequestPriority.NORMAL,
            15*1000,
            new StateRequest<>(ShootingStateRequest.SHOOT_PRELOADS, StateRequestPriority.NORMAL, 3*1000, StateRequestGroupChildTimeoutBehavior.SKIP),
            new CMStateRequest(new CMTrajectory(
                "startPosition_HumanPlayerTrench_Wall_L_In", 
                Paths.startPosition_HumanPlayerTrench_Wall_L_In.pathPoints(), 
                new CMRotation[] {Paths.startPosition_HumanPlayerTrench_Wall_L_In.getFirstRotation(1)}, // TODO: prolly want to specify accel decel whatnot (and add other constructors)
                new CMEvent[] {}, 
                TrajectoryPriority.PREFER_TRANSLATION, 
                false, // TODO: Since not stopping, use all parameters of CMTrajectory
                new double[] {0.5, 0.5}, 
                0,
                15*1000)),
            new CMStateRequest(new CMTrajectory(
                "startPosition_HumanPlayerTrench_Wall_L_Out", 
                Paths.startPosition_HumanPlayerTrench_Wall_L_Out.pathPoints(), 
                new CMRotation[] {Paths.startPosition_HumanPlayerTrench_Wall_L_Out.getFirstRotation(1)}, // TODO: prolly want to specify accel decel whatnot (and add other constructors)
                new CMEvent[] {}, 
                TrajectoryPriority.PREFER_TRANSLATION, 
                false, // TODO: Since not stopping, use all parameters of CMTrajectory
                new double[] {0.5, 0.5}, 
                0,
                15*1000)),
            new StateRequest<>(ShootingStateRequest.SHOOT_PRELOADS, StateRequestPriority.NORMAL, 8*1000, StateRequestGroupChildTimeoutBehavior.SKIP)

        );

        };
}
