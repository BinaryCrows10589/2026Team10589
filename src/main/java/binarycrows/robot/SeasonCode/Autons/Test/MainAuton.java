package binarycrows.robot.SeasonCode.Autons.Test;

import binarycrows.robot.StateRequest;
import binarycrows.robot.CrowMotion.UserSide.CMStateRequest;
import binarycrows.robot.SeasonCode.Autons.Trajectories.Trajectories;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class MainAuton {
    // TODO: Acutally add the autos and please make them not inside of the test folder. 
    // Each main auto should have a folder. Also dont name them Main, secendery ect. 
    // Name them according to what they do. So ShootPreLoad_ShootCenter_ClimbRight ect
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


