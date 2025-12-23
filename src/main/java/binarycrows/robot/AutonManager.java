package binarycrows.robot;

import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.SeasonCode.Autons.Test.CMTest1;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveSubStateManager;

public class AutonManager {
    public static void runAuton() {
        CMTest1.initialize();
        (new StateRequest<DriveStateRequest>(DriveStateRequest.DRIVE_CROWMOTION, StateRequestPriority.NORMAL)).dispatchSelf();
    }
}
