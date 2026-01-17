package binarycrows.robot.SeasonCode.Autons.Test;

import java.util.function.Supplier;

import binarycrows.robot.StateRequest;
import binarycrows.robot.CrowMotion.UserSide.CMAutonPoint;
import binarycrows.robot.CrowMotion.UserSide.CMEvent;
import binarycrows.robot.CrowMotion.UserSide.CMRotation;
import binarycrows.robot.CrowMotion.UserSide.CMStateRequest;
import binarycrows.robot.CrowMotion.UserSide.CMTrajectory;
import binarycrows.robot.CrowMotion.UserSide.CMTrajectory.TrajectoryPriority;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.SeasonCode.Constants.CrowMotionConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveSubStateManager;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class CMTest2 {
    public static Pair<Pose2d,StateRequest[]> getPair() {
        return new Pair<>(startingPoint, getAutonomous());
    }
    public static Pose2d startingPoint = new Pose2d(0, 0, Rotation2d.kZero);

    public static StateRequest[] getAutonomous() {
        return new StateRequest[] {
            new CMStateRequest(
                new CMTrajectory(
                    "TestTraj1",
                        new CMAutonPoint[] {
                            new CMAutonPoint(0,0, false),
                            new CMAutonPoint(2, -1, false),
                            new CMAutonPoint(100, 0, false)
                        }, 
                        new CMRotation[] {
                            new CMRotation(0, 
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
                            new double[] {.01, .01}, .04, 50)),
                
                    new CMStateRequest(
                        new CMTrajectory(
                    "TestTraj2",
                        new CMAutonPoint[] {
                            new CMAutonPoint(100,0, false),
                            new CMAutonPoint(2, -1, false),
                            new CMAutonPoint(0, 0, false)
                        }, 
                        new CMRotation[] {
                            new CMRotation(0, 
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
                            new double[] {.01, .01}, .04, 50)),
                };
        };
}


