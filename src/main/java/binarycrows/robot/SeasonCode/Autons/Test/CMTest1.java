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
        CrowMotionConstants.currentTrajectory = new CMTrajectory("CMTest1",
                new CMAutonPoint[] {
                    new CMAutonPoint(0, 0, false),
                    new CMAutonPoint(2, 2, false),
                    new CMAutonPoint(1.5, 3, false)
                }, 
                new CMRotation[] {
                    new CMRotation(10, 1, .25, 1, 5, .5, 10, 1),
                   
                    }, 
                    null,
                    TrajectoryPriority.SPLIT_PROPORTIONALLY,
                    4.4, 4.4, 4.4, 0,
                    true,
                    new double[] {.01, .01}, .04, 50
                );
    }
}
