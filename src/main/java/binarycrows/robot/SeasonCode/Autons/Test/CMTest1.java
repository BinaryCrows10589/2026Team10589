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

public class CMTest1 {
    public static void initialize() {
        DriveSubStateManager.getInstance().setRobotStartingPose(new AutonPoint(0, 0, 0, false));
        CrowMotionConstants.currentTrajectory = new CMTrajectory("TestTraj",
        new CMAutonPoint[] {
            new CMAutonPoint(0, 0, false),
            new CMAutonPoint(6, 0, false),
            //new CMAutonPoint(3, 1.5, false)
        },
        new CMRotation[] {
            new CMRotation(0, 1, .25, 1, 5, .5, 10, 1),
            }, 
            null,
            /*new CMEvent[] {
                new CMEvent("goBaack", () -> {
                    CrowMotionConstants.currentTrajectory = new CMTrajectory("TestTrajBack",
                        new CMAutonPoint[] {
                            new CMAutonPoint(0, 0, false),
                            new CMAutonPoint(2, 0, false),
                            //new CMAutonPoint(3, 1.5, false)
                        },
                        new CMRotation[] {
                            new CMRotation(0, 1, .25, 1, 5, .5, 10, 1),
                            }, 
                            null,
                            3,
                            10,
                            TrajectoryPriority.SPLIT_PROPORTIONALLY,
                            1.5,
                            1,
                            1,
                            0.5,
                            true,
                            .05,
                            .1,
                            new double[] {0.01, 0.01},
                            .04, 50  
                        );
                        (new StateRequest<DriveStateRequest>(DriveStateRequest.DRIVE_CROWMOTION, StateRequestPriority.NORMAL)).dispatchSelf();

                }, 1)
            },*/
            3,
            10,
            TrajectoryPriority.SPLIT_PROPORTIONALLY,
            1.5,
            1,
            1,
            0.5,
            true,
            .05,
            .1,
            new double[] {0.01, 0.01},
            .04, 50  
        );
    }
}
