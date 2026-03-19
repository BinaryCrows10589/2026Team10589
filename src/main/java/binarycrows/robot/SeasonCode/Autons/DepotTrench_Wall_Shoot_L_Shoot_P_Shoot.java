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

public class DepotTrench_Wall_Shoot_L_Shoot_P_Shoot {
    public static Pose2d startingPoint = new Pose2d(Points.startPosition_DepotTrench_Wall.geTranslation2d(), Rotation2d.fromDegrees(-90));

    public static SequentialGroup getAutonomous() {
        return new SequentialGroup(
            StateRequestPriority.NORMAL,
            15*1000,

            // TODO: If time, add all params
            
            // Shoot Preloads
            new StateRequest<>(ShootingStateRequest.SHOOT_PRELOADS, StateRequestPriority.NORMAL, 3*1000, StateRequestGroupChildTimeoutBehavior.SKIP),
            
            // L In
            new CMStateRequest(new CMTrajectory(
                "startPosition_DepotTrench_Wall_L_In", 
                Paths.startPosition_DepotTrench_Wall_L_In.pathPoints(), 
                new CMRotation[] {Paths.startPosition_DepotTrench_Wall_L_In.getFirstRotation(1)}, // TODO: prolly want to specify accel decel whatnot (and add other constructors)
                new CMEvent[] {}, 
                1.5,
                6,
                TrajectoryPriority.SPLIT_PROPORTIONALLY,
                3.5,
                3.5,
                3.5,
                2.0,
                false,
                2,
                0,
                new double[] {0.5, 0.5}, 
                0.04,
                15*1000)),

            // L Out
            new CMStateRequest(new CMTrajectory(
                "startPosition_DepotTrench_Wall_L_Out", 
                Paths.DepotSide_L_Out_Trench.pathPoints(), 
                new CMRotation[] {Paths.DepotSide_L_Out_Trench.getFirstRotation(1)}, // TODO: prolly want to specify accel decel whatnot (and add other constructors)
                new CMEvent[] {}, 
                1.5,
                9,
                TrajectoryPriority.SPLIT_PROPORTIONALLY, 
                3.5,
                3.5,
                3.5,
                2.0,
                false, // TODO: Since not stopping, use all parameters of CMTrajectory
                2,
                0,
                new double[] {0.5, 0.5}, 
                0.04,
                15*1000))
            
            // Shoot All
            //new StateRequest<>(ShootingStateRequest.SHOOT_PRELOADS, StateRequestPriority.NORMAL, 8*1000, StateRequestGroupChildTimeoutBehavior.SKIP)
            
            /*
            // P In
            new CMStateRequest(new CMTrajectory(
                "startPosition_DepotTrench_Wall_P_In", 
                Paths.startPosition_DepotTrench_Wall_P_In.pathPoints(), 
                new CMRotation[] {Paths.startPosition_DepotTrench_Wall_P_In.getFirstRotation(1)},
                new CMEvent[] {}, 
                1.5,
                10,
                TrajectoryPriority.PREFER_TRANSLATION, 
                3.5,
                3.5,
                3.5,
                2,
                false,
                1,
                0,
                new double[] {0.5, 0.5}, 
                0,
                15*1000)),
            new CMStateRequest(new CMTrajectory(
                "startPosition_DepotTrench_Wall_P_Arc", 
                Paths.startPosition_DepotTrench_Wall_P_Arc.pathPoints(), 
                new CMRotation[] {
                    new CMRotation(90, 1, 0.2, 5),
                    new CMRotation(270, -1, 0.9, 5)
                },
                new CMEvent[] {}, 
                1.5,
                5,
                TrajectoryPriority.PREFER_TRANSLATION, 
                1,
                1,
                3.5,
                0.5,
                false,
                1,
                0,
                new double[] {0.05, 0.05}, 
                0.04,
                15*1000))
                */
            /* 
            // P Out
            new CMStateRequest(new CMTrajectory(
                "startPosition_DepotTrench_Wall_P_Out", 
                Paths.DepotSide_P_Out_Trench.pathPoints(), 
                new CMRotation[] {Paths.DepotSide_P_Out_Trench.getFirstRotation(1)}, // TODO: prolly want to specify accel decel whatnot (and add other constructors)
                new CMEvent[] {}, 
                TrajectoryPriority.PREFER_TRANSLATION, 
                false, // TODO: Since not stopping, use all parameters of CMTrajectory
                new double[] {0.5, 0.5}, 
                0,
                15*1000)),

            // Shoot All
                new StateRequest<>(ShootingStateRequest.SHOOT_PRELOADS, StateRequestPriority.NORMAL, 8*1000, StateRequestGroupChildTimeoutBehavior.SKIP)
                */
            );


        };
}
