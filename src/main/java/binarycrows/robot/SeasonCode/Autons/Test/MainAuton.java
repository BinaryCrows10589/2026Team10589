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
import binarycrows.robot.SeasonCode.Autons.Trajectories.Trajectories;
import binarycrows.robot.SeasonCode.Constants.CrowMotionConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveSubStateManager;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class MainAuton {
    public static Pair<Pose2d,StateRequest[]> getPair() {
        return new Pair<>(startingPoint, getAutonomous());
    }
    public static Pose2d startingPoint = new Pose2d(3.6, 5.5, Rotation2d.fromDegrees(-90));

    public static StateRequest[] getAutonomous() {
        return new StateRequest[] {
            new CMStateRequest(Trajectories.getStartOfRampToBallLineUp()),
            new CMStateRequest(Trajectories.BallLineUpToMiddle()),
            new CMStateRequest(Trajectories.MiddleToSharpReturn())
                
            
        
        };

        };
}


