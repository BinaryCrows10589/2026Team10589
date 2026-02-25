package binarycrows.robot.SeasonCode.Autons;

import binarycrows.robot.CrowMotion.UserSide.CMEvent;
import binarycrows.robot.CrowMotion.UserSide.CMRotation;
import binarycrows.robot.CrowMotion.UserSide.CMStateRequest;
import binarycrows.robot.CrowMotion.UserSide.CMTrajectory;
import binarycrows.robot.CrowMotion.UserSide.CMTrajectory.TrajectoryPriority;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.SeasonCode.Autons.Data.Paths;
import binarycrows.robot.SeasonCode.Autons.Data.Points;
import binarycrows.robot.StateRequestGroup.SequentialGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class StartPosition_HumanPlayerRamp_Trench_ShootPreloads {
    public static Pose2d startingPoint = new Pose2d(Points.startPosition_HumanPlayerRamp_Trench.geTranslation2d(), Rotation2d.fromDegrees(-90));

    public static SequentialGroup getAutonomous() {
        return new SequentialGroup(
            StateRequestPriority.NORMAL,
            15*1000
            // TODO: PUT SHOOTING PRELOADS STATE REQUEST HERE!!!!!!
        );

        };
}
