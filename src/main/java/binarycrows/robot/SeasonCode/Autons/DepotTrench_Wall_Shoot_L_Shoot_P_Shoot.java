package binarycrows.robot.SeasonCode.Autons;

import javax.naming.spi.StateFactory;

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
import binarycrows.robot.SeasonCode.SubStateManagers.Intake.Rollers.IntakeRollersStateRequest;
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
                new CMRotation[] {new CMRotation(90, 0, 0.25, 5), new CMRotation(110, 0, 1, 5)},
                new CMEvent[] {new CMEvent("deployIntake", new StateRequest<>(IntakeRollersStateRequest.OVERDRIVE, StateRequestPriority.NORMAL)::dispatchSelf, 0.5)}, 
                4,
                10,
                TrajectoryPriority.SPLIT_PROPORTIONALLY,
                1.5,
                1.5,
                4.4,
                2,
                false,
                1,
                0,
                new double[] {0.5, 0.5},
                0.04,
                15*1000)),
            
            new CMStateRequest(new CMTrajectory(
                "startPosition_DepotTrench_Wall_Arch_Half_One", 
                Paths.startPosition_DepotTrench_Wall_L_Arch_Half_One.pathPoints(), 
                new CMRotation[] {new CMRotation(-90, -1, .5, 20)},
                new CMEvent[] {}, 
                4,
                15,
                TrajectoryPriority.SPLIT_PROPORTIONALLY,
                1.5,
                1.5,
                4.4,
                1,
                false,
                1,
                0,
                new double[] {0.9, 0.9},
                0,
                15*1000)),

            new StateRequest<>(IntakeRollersStateRequest.INTAKING, StateRequestPriority.NORMAL),


            /*new CMStateRequest(new CMTrajectory(
                "startPosition_DepotTrench_Wall_Arch_Half_Two", 
                Paths.startPosition_DepotTrench_Wall_L_Arch_Half_Two.pathPoints(), 
                new CMRotation[] {new CMRotation(-90, -1, .5, 5)},
                new CMEvent[] {}, 
                4,
                15,
                TrajectoryPriority.SPLIT_PROPORTIONALLY,
                1.5,
                1.5,
                4.4,
                2,
                false,
                1,
                0,
                new double[] {0.5, 0.5},
                0.04,
                15*1000)),*/

            new CMStateRequest(new CMTrajectory(
                "startPosition_DepotTrench_Wall_Return", 
                Paths.startPosition_DepotTrench_Wall_L_Return.pathPoints(), 
                new CMRotation[] {new CMRotation(-125, 0, .5, 5)},
                new CMEvent[] {}, 
                4,
                15,
                TrajectoryPriority.SPLIT_PROPORTIONALLY,
                1.5,
                1.5,
                4.4,
                1,
                true,
                .25,
                0.1,
                new double[] {0.2, 0.2},
                0.04,
                15*1000)),

            // Cycle 2

            new StateRequest<>(ShootingStateRequest.SHOOT, StateRequestPriority.NORMAL),
            
            new CMStateRequest(new CMTrajectory(
                "startPosition_DepotTrench_Wall_In_Second_Crawl", 
                Paths.startPosition_DepotTrench_Wall_L_In_Second_Crawl.pathPoints(), 
                new CMRotation[] {new CMRotation(-180, 0, .5, 2)},
                new CMEvent[] {}, 
                4,
                10,
                TrajectoryPriority.SPLIT_PROPORTIONALLY,
                0.3,
                0.2,
                4.4,
                2,
                false,
                .1,
                0,
                new double[] {0.5, 0.5},
                0.04,
                15*1000)),

            new StateRequest<>(ShootingStateRequest.SHOOT_PRELOADS, StateRequestPriority.NORMAL, 8*1000, StateRequestGroupChildTimeoutBehavior.SKIP),

            new CMStateRequest(new CMTrajectory(
                "startPosition_DepotTrench_Wall_In_Second", 
                Paths.startPosition_DepotTrench_Wall_L_In_Second.pathPoints(), 
                new CMRotation[] {new CMRotation(-180, 0, .25, 10)},
                new CMEvent[] {}, 
                4,
                10,
                TrajectoryPriority.SPLIT_PROPORTIONALLY,
                1.5,
                1.5,
                4.4,
                2,
                false,
                1,
                0,
                new double[] {0.5, 0.5},
                0.04,
                15*1000)),

            new StateRequest<>(IntakeRollersStateRequest.OVERDRIVE, StateRequestPriority.NORMAL),


            new CMStateRequest(new CMTrajectory(
                "startPosition_DepotTrench_Wall_Insertion", 
                Paths.startPosition_DepotTrench_Wall_L_Insertion.pathPoints(), 
                new CMRotation[] {new CMRotation(90, 0, .5, 3)},
                new CMEvent[] {}, 
                4,
                15,
                TrajectoryPriority.SPLIT_PROPORTIONALLY,
                1.5,
                1.5,
                4.4,
                2,
                true,
                .25,
                0.1,
                new double[] {0.01, 0.01},
                0.04,
                15*1000)),

            new StateRequest<>(IntakeRollersStateRequest.INTAKING, StateRequestPriority.NORMAL),

                
            new CMStateRequest(new CMTrajectory(
                "startPosition_DepotTrench_Wall_L_Retraction", 
                Paths.startPosition_DepotTrench_Wall_L_Retraction.pathPoints(), 
                new CMRotation[] {new CMRotation(90, 0, .5, 2)},
                new CMEvent[] {}, 
                4,
                10,
                TrajectoryPriority.SPLIT_PROPORTIONALLY,
                1.5,
                1.5,
                4.4,
                2,
                false,
                1,
                0,
                new double[] {0.5, 0.5},
                0.04,
                15*1000)),



            new CMStateRequest(new CMTrajectory(
                "startPosition_DepotTrench_Wall_Return_Second", 
                Paths.startPosition_DepotTrench_Wall_L_Return_Second.pathPoints(), 
                new CMRotation[] {new CMRotation(90, 0, .5, 8)},
                new CMEvent[] {}, 
                4,
                15,
                TrajectoryPriority.SPLIT_PROPORTIONALLY,
                2.25,
                2.25,
                4.4,
                2,
                true,
                .25,
                0.1,
                new double[] {0.5, 0.5},
                0.04,
                15*1000)),

            new StateRequest<>(ShootingStateRequest.SHOOT, StateRequestPriority.NORMAL),

            
            new CMStateRequest(new CMTrajectory(
                "startPosition_DepotTrench_Wall_L_Second_Crawl", 
                Paths.startPosition_DepotTrench_Wall_L_Second_Crawl.pathPoints(), 
                new CMRotation[] {},
                new CMEvent[] {}, 
                4,
                10,
                TrajectoryPriority.SPLIT_PROPORTIONALLY,
                0.3,
                0.2,
                4.4,
                2,
                false,
                .1,
                0,
                new double[] {0.5, 0.5},
                0.04,
                15*1000)),

            // Shoot All
            new StateRequest<>(ShootingStateRequest.SHOOT_PRELOADS, StateRequestPriority.NORMAL, 8*1000, StateRequestGroupChildTimeoutBehavior.SKIP)


        );


        };
}
