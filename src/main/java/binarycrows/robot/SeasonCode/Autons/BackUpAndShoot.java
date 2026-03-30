package binarycrows.robot.SeasonCode.Autons;

import binarycrows.robot.StateRequest;
import binarycrows.robot.CrowMotion.UserSide.CMAutonPoint;
import binarycrows.robot.CrowMotion.UserSide.CMEvent;
import binarycrows.robot.CrowMotion.UserSide.CMRotation;
import binarycrows.robot.CrowMotion.UserSide.CMStateRequest;
import binarycrows.robot.CrowMotion.UserSide.CMTrajectory;
import binarycrows.robot.CrowMotion.UserSide.CMTrajectory.TrajectoryPriority;
import binarycrows.robot.Enums.StateRequestGroupChildTimeoutBehavior;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.SeasonCode.Autons.Data.Paths;
import binarycrows.robot.SeasonCode.Autons.Data.Points;
import binarycrows.robot.SeasonCode.SubStateManagers.Intake.Pivot.PivotStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.Intake.Rollers.IntakeRollersStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.Shooting.ShootingStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveStateRequest;
import binarycrows.robot.StateRequestGroup.SequentialGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class BackUpAndShoot {
    public static Pose2d startingPoint = new Pose2d(3.547, 4.016, Rotation2d.fromDegrees(0));


    public static SequentialGroup getAutonomous() {

        return new SequentialGroup(
            StateRequestPriority.NORMAL,
            15*1000,

        new CMStateRequest(new CMTrajectory(
                "backUpAndShoot", 
                new CMAutonPoint[] {
                    new CMAutonPoint(3.547, 4.016),
                    new CMAutonPoint(3.547-2, 4.016),
                }, 
                new CMRotation[] {new CMRotation(0, 0, 1, 5)},
                new CMEvent[] {
                }, 
                4,
                10,
                TrajectoryPriority.SPLIT_PROPORTIONALLY,
                2,
                2,
                4,
                2,
                true,
                .1,
                .25,
                new double[] {0.02, 0.02},
                0.04,
                15*1000)),

                new StateRequest<>(ShootingStateRequest.FORCE_SHOOT, StateRequestPriority.NORMAL)
        );


        };
}
