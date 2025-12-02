package binarycrows.robot.SeasonCode.Autons.Test;

import java.util.function.Supplier;

import binarycrows.robot.CrowMotion.UserSide.CMAutonPoint;
import binarycrows.robot.CrowMotion.UserSide.CMRotation;
import binarycrows.robot.CrowMotion.UserSide.CMTrajectory;
import binarycrows.robot.CrowMotion.UserSide.CMTrajectory.TrajectoryPriority;
import binarycrows.robot.SeasonCode.Constants.CrowMotionConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveSubStateManager;
import binarycrows.robot.Utils.Auton.AutonPoint;

public class CMTest1 {
    public static void initialize() {
        DriveSubStateManager.getInstance().setRobotStartingPose(new AutonPoint(0, 0, 0, false));
        CrowMotionConstants.currentTrajectory = new CMTrajectory("TestTraj",
        new CMAutonPoint[] {
            new CMAutonPoint(0, 0, false),
            new CMAutonPoint(2, 2, false),
            new CMAutonPoint(3, 1.5, false)
        }, 
        new CMRotation[] {
            new CMRotation(10, 1, .25, 1, 5, .5, 10, 1),
            }, 
            null,
            3,
            10,
            TrajectoryPriority.SPLIT_PROPORTIONALLY,
            3.5,
            3.5,
            3.5,
            2,
            true,
            .05,
            .1,
            new double[] {0.01, 0.01},
            .04, 50  
        );
    }
}
