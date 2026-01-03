package binarycrows.robot.CrowMotion.UserSide;

import binarycrows.robot.StateRequest;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveStateRequest;

public class CMStateRequest extends StateRequest<DriveStateRequest> {

    public CMTrajectory trajectory;

    public CMStateRequest(CMTrajectory trajectory) {
        super(DriveStateRequest.DRIVE_CROWMOTION, StateRequestPriority.NORMAL);
        this.trajectory = trajectory;
    }
    
}
