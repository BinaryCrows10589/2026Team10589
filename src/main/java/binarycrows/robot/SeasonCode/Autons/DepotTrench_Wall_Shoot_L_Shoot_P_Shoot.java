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
import binarycrows.robot.SeasonCode.SubStateManagers.Intake.Rollers.IntakeRollersStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.Shooting.ShootingStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveStateRequest;
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



            new CMStateRequest(new CMTrajectory(
                "startPosition_DepotTrench_Wall_L_In", 
                Paths.startPosition_DepotTrench_Wall_L_In.pathPoints(), 
                new CMRotation[] {new CMRotation(90, 0, 0.25, 5), new CMRotation(110, 0, 1, 5)},
                new CMEvent[] {
                    new CMEvent("deployIntake", new StateRequest<>(PivotStateRequest.DOWN, StateRequestPriority.NORMAL)::dispatchSelf, 0.25),

                    new CMEvent("overdrive", new StateRequest<>(IntakeRollersStateRequest.OVERDRIVE, StateRequestPriority.NORMAL)::dispatchSelf, 0.5)
                }, 
                4,
                10,
                TrajectoryPriority.SPLIT_PROPORTIONALLY,
                4.6,
                4.6,
                4.6,
                2,
                false,
                4.6,
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
                4.6,
                4.6,
                4.4,
                1,
                false,
                4.6,
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
                2.25,
                2.25,
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
                2.5,
                2.5,
                4.4,
                1,
                true,
                2,
                0.1,
                new double[] {0.2, 0.2},
                0.04,
                15*1000)),

            // Cycle 2

            new StateRequest<>(IntakeRollersStateRequest.OFF, StateRequestPriority.NORMAL),
            new StateRequest<>(ShootingStateRequest.FORCE_SHOOT, StateRequestPriority.NORMAL),
            
            new CMStateRequest(new CMTrajectory(
                "startPosition_DepotTrench_Wall_In_Second_Crawl", 
                Paths.startPosition_DepotTrench_Wall_L_In_Second_Crawl.pathPoints(), 
                new CMRotation[] {
                    new CMRotation(-200, 0, .05, 25, 25, 25,  0.01),
                    new CMRotation(-160, 0, .15, 25, 25, 25,  0.01),
                    new CMRotation(-200, 0, .25, 25, 25, 25,  0.01),
                    new CMRotation(-160, 0, .35, 25, 25, 25,  0.01),
                    new CMRotation(-200, 0, .45, 25, 25, 25,  0.01),
                    new CMRotation(-160, 0, .55, 25, 25, 25,  0.01),
                    new CMRotation(-200, 0, .65, 25, 25, 25,  0.01),
                    new CMRotation(-160, 0, .75, 25, 25, 25,  0.01),
                    new CMRotation(-200, 0, .85, 25, 25, 25,  0.01),
                    new CMRotation(-160, 0, .95, 25, 25, 25,  0.01),
                    new CMRotation(-200, 0, 1, 2)
                },
                new CMEvent[] {
                    new CMEvent("raiseIntake1", new StateRequest<>(PivotStateRequest.RAISED, StateRequestPriority.NORMAL)::dispatchSelf, 0.25),
                    new CMEvent("lowerIntake1", new StateRequest<>(PivotStateRequest.DOWN, StateRequestPriority.NORMAL)::dispatchSelf, 0.5),
                    new CMEvent("raiseIntake2", new StateRequest<>(PivotStateRequest.RAISED, StateRequestPriority.NORMAL)::dispatchSelf, 0.75),
                    new CMEvent("lowerIntake2", new StateRequest<>(PivotStateRequest.DOWN, StateRequestPriority.NORMAL)::dispatchSelf, 0.8)


                }, 
                4,
                10,
                TrajectoryPriority.SPLIT_PROPORTIONALLY,
                0.3,
                0.2,
                4.4,
                0.5,
                false,
                .1,
                0,
                new double[] {0.5, 0.5},
                0,
                15*1000)),

            new StateRequest<>(ShootingStateRequest.SHOOT_PRELOADS, StateRequestPriority.NORMAL, 8*1000, StateRequestGroupChildTimeoutBehavior.SKIP),
            
            new StateRequest<>(IntakeRollersStateRequest.INTAKING, StateRequestPriority.NORMAL),

            new CMStateRequest(new CMTrajectory(
                "startPosition_DepotTrench_Wall_In_Second", 
                Paths.startPosition_DepotTrench_Wall_L_In_Second.pathPoints(), 
                new CMRotation[] {
                    new CMRotation(-200, 0, .5, 10),
                    new CMRotation(90, 0, .9, 10)},
                new CMEvent[] {}, 
                4,
                10,
                TrajectoryPriority.SPLIT_PROPORTIONALLY,
                3.75,
                3.75,
                4.4,
                2,
                false,
            0.25,
                0,
                new double[] {0.5, 0.5},
                0.04,
                15*1000)),

            new StateRequest<>(IntakeRollersStateRequest.OVERDRIVE, StateRequestPriority.NORMAL),


            new CMStateRequest(new CMTrajectory(
                "startPosition_DepotTrench_Wall_Arch_Half_One_Second", 
                Paths.startPosition_DepotTrench_Wall_L_Arch_Half_One_Second.pathPoints(), 
                new CMRotation[] {new CMRotation(180, 0, .5, 20)},
                new CMEvent[] {}, 
                4,
                15,
                TrajectoryPriority.SPLIT_PROPORTIONALLY,
                4,
                4,
                4.4,
                1,
                false,
                4,
                0,
                new double[] {0.9, 0.9},
                0,
                15*1000)),

            new StateRequest<>(IntakeRollersStateRequest.INTAKING, StateRequestPriority.NORMAL),




            new CMStateRequest(new CMTrajectory(
                "startPosition_DepotTrench_Wall_Return_Second", 
                Paths.startPosition_DepotTrench_Wall_L_Return.pathPoints(), 
                new CMRotation[] {new CMRotation(180, 0, .5, 8)},
                new CMEvent[] {}, 
                4,
                15,
                TrajectoryPriority.SPLIT_PROPORTIONALLY,
                2.5,
                2.5,
                4.4,
                2,
                true,
                2,
                0.1,
                new double[] {0.5, 0.5},
                0.04,
                15*1000)),

            new StateRequest<>(ShootingStateRequest.FORCE_SHOOT, StateRequestPriority.NORMAL),

            
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
                .5,
                false,
                .1,
                0,
                new double[] {0.5, 0.5},
                0.04,
                15*1000)),

            new StateRequest<>(DriveStateRequest.DISABLE, StateRequestPriority.NORMAL),

            // Shoot All
            new StateRequest<>(ShootingStateRequest.SHOOT_PRELOADS, StateRequestPriority.NORMAL, 8*1000, StateRequestGroupChildTimeoutBehavior.SKIP)


        );


        };
}
