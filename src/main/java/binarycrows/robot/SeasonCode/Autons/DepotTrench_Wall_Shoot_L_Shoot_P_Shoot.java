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
import binarycrows.robot.SeasonCode.SubStateManagers.Intake.Pivot.PivotStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.Shooting.ShootingStateRequest;
import binarycrows.robot.StateRequestGroup.SequentialGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class DepotTrench_Wall_Shoot_L_Shoot_P_Shoot {
    public static Pose2d startingPoint = new Pose2d(Points.startPosition_DepotTrench_Wall.getTranslation2d(), Rotation2d.fromDegrees(90));

    public static SequentialGroup getAutonomous() {

        return new SequentialGroup(
            StateRequestPriority.NORMAL,
            15*1000,
            
            // First In

            new StateRequest<>(PivotStateRequest.DOWN, StateRequestPriority.NORMAL),

            new CMStateRequest(new CMTrajectory(
                "startPosition_DepotTrench_Wall_L_In", 
                Paths.startPosition_DepotTrench_Wall_L_In.pathPoints(), 
                new CMRotation[] {},
                new CMEvent[] {}, 
                4,
                10,
                TrajectoryPriority.SPLIT_PROPORTIONALLY,
                4.4,
                4.4,
                4.4,
                2,
                false,
                1,
                0,
                new double[] {0.5, 0.5},
                0.04,
                15*1000)),
            
            new CMStateRequest(new CMTrajectory(
                "startPosition_DepotTrench_Wall_Arch", 
                Paths.startPosition_DepotTrench_Wall_L_Arch.pathPoints(), 
                new CMRotation[] {new CMRotation(-45, 0, .5, 1)},
                new CMEvent[] {}, 
                4,
                15,
                TrajectoryPriority.SPLIT_PROPORTIONALLY,
                3,
                4.4,
                4.4,
                2,
                false,
                1,
                0,
                new double[] {0.5, 0.5},
                0.04,
                15*1000)),

            new CMStateRequest(new CMTrajectory(
                "startPosition_DepotTrench_Wall_Return", 
                Paths.startPosition_DepotTrench_Wall_L_Return.pathPoints(), 
                new CMRotation[] {new CMRotation(-45, 0, .5, 1)},
                new CMEvent[] {}, 
                4,
                15,
                TrajectoryPriority.SPLIT_PROPORTIONALLY,
                3,
                4.4,
                4.4,
                2,
                true,
                .25,
                0.1,
                new double[] {0.01, 0.01},
                0.04,
                15*1000)),
            
            // Shoot All
            new StateRequest<>(ShootingStateRequest.SHOOT_PRELOADS, StateRequestPriority.NORMAL, 8*1000, StateRequestGroupChildTimeoutBehavior.SKIP)



        );


        };
}
