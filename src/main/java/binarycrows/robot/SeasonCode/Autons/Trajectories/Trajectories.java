package binarycrows.robot.SeasonCode.Autons.Trajectories;

import binarycrows.robot.CrowMotion.UserSide.CMAutonPoint;
import binarycrows.robot.CrowMotion.UserSide.CMRotation;
import binarycrows.robot.CrowMotion.UserSide.CMTrajectory;
import binarycrows.robot.CrowMotion.UserSide.CMTrajectory.TrajectoryPriority;

public class Trajectories {
    public static CMTrajectory getStartOfRampToBallLineUp() {
        return new CMTrajectory(
        "StartOfRampToBallLineUp",
            new CMAutonPoint[] {
                new CMAutonPoint(3.6,5.5, false),
                new CMAutonPoint(6.65, 5.5, false),
                new CMAutonPoint(7.725, 7.2, false)
            }, 
            new CMRotation[] {
                new CMRotation(90, 
                1, 
                1,
                5, 
                5, 
                .5, 
                10, 
                1),
            
                }, 
                null,
                4,
                8,
                TrajectoryPriority.SPLIT_PROPORTIONALLY,
                4,
                4,
                3,
                2.5,
                true,
                .05,
                .01,
                new double[] {.01, .01}, .04, 50);
    }
    public static CMTrajectory BallLineUpToMiddle() {
        return new CMTrajectory(
        "BallLineUpToMiddle",
            new CMAutonPoint[] {
                new CMAutonPoint(7.725, 7.2, false),
                new CMAutonPoint(7.725, 4, false),
            }, 
            new CMRotation[] {
                new CMRotation(90, 
                1, 
                1,
                5, 
                5, 
                .5, 
                10, 
                1),
            
                }, 
                null,
                4,
                8,
                TrajectoryPriority.SPLIT_PROPORTIONALLY,
                4,
                4,
                3,
                2.5,
                true,
                .05,
                .01,
                new double[] {.05, .05}, 
                .04, 50);
    }
    public static CMTrajectory MiddleToSharpReturn() {
        return new CMTrajectory(
        "MiddleToSharpReturn",
            new CMAutonPoint[] {
                new CMAutonPoint(7.725, 4, false),
                new CMAutonPoint(6.725, 6, false),
                new CMAutonPoint(4.6, 6, false),
                new CMAutonPoint(0.725, 5.25, false),

            }, 
            new CMRotation[] {
                new CMRotation(90, 
                1, 
                1,
                5, 
                5, 
                .5, 
                10, 
                1),
            
                }, 
                null,
                4,
                8,
                TrajectoryPriority.SPLIT_PROPORTIONALLY,
                4,
                4,
                3,
                2.5,
                true,
                .05,
                .01,
                new double[] {.01, .01}, .04, 50);
    }
}
