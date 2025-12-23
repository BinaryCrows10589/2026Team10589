package binarycrows.robot.SeasonCode.Autons.Test;

import java.util.function.Supplier;

import binarycrows.robot.StateRequest;
import binarycrows.robot.CrowMotion.UserSide.CMAutonPoint;
import binarycrows.robot.CrowMotion.UserSide.CMEvent;
import binarycrows.robot.CrowMotion.UserSide.CMRotation;
import binarycrows.robot.CrowMotion.UserSide.CMTrajectory;
import binarycrows.robot.CrowMotion.UserSide.CMTrajectory.TrajectoryPriority;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.SeasonCode.Constants.CrowMotionConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveSubStateManager;
import binarycrows.robot.Utils.Auton.AutonPoint;
import edu.wpi.first.math.geometry.Pose2d;

public class CMTest1 {
    public static void initialize() {
        DriveSubStateManager.getInstance().setRobotStartingPose(new AutonPoint(0, 0, 0, false));
        CrowMotionConstants.currentTrajectory = 
        new CMTrajectory(
            "TestTraj",
                new CMAutonPoint[] {
                    new CMAutonPoint(0,0, false),
                    new CMAutonPoint(2, -1, false),
                    new CMAutonPoint(5.09, 0, false)
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
                    new double[] {.01, .01}, .04, 50
                );
    }
}


