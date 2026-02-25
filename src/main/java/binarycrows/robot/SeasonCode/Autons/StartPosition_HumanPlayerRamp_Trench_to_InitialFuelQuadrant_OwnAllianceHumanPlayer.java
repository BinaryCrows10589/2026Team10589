package binarycrows.robot.SeasonCode.Autons;

import static edu.wpi.first.units.Units.Rotation;

import java.util.ArrayList;

import binarycrows.robot.StateRequest;
import binarycrows.robot.CrowMotion.UserSide.CMTrajectory;
import binarycrows.robot.CrowMotion.UserSide.CMTrajectory.TrajectoryPriority;
import binarycrows.robot.CrowMotion.UserSide.CMAutonPoint;
import binarycrows.robot.CrowMotion.UserSide.CMEvent;
import binarycrows.robot.CrowMotion.UserSide.CMRotation;
import binarycrows.robot.CrowMotion.UserSide.CMStateRequest;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.SeasonCode.Autons.Data.Paths;
import binarycrows.robot.SeasonCode.Autons.Data.Points;
import binarycrows.robot.StateRequestGroup.SequentialGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class StartPosition_HumanPlayerRamp_Trench_to_InitialFuelQuadrant_OwnAllianceHumanPlayer {
    public static Pose2d startingPoint = new Pose2d(Points.startPosition_HumanPlayerRamp_Trench.geTranslation2d(), Rotation2d.fromDegrees(-90));

    public static SequentialGroup getAutonomous() {
        return new SequentialGroup(
            StateRequestPriority.NORMAL,
            15*1000,
            StartPosition_HumanPlayerRamp_Trench_ShootPreloads.getAutonomous(),
            new CMStateRequest(new CMTrajectory(
                "StartPosition_HumanPlayerRamp_Trench_to_InitialFuelQuadrant_OwnAllianceHumanPlayer", 
                Paths.startPosition_HumanPlayerRamp_Trench_to_InitialFuelQuadrant_OwnAllianceHumanPlayer.pathPoints(), 
                new CMRotation[] {Paths.startPosition_HumanPlayerRamp_Trench_to_InitialFuelQuadrant_OwnAllianceHumanPlayer.pathRotations()[0].createCMRotation(1)}, // TODO: prolly want to specify accel decel whatnot (and add other constructors)
                new CMEvent[] {}, 
                TrajectoryPriority.PREFER_TRANSLATION, 
                false, // TODO: Since not stopping, use all parameters of CMTrajectory
                new double[] {0.5, 0.5}, 
                0,
                15*1000))
        );

        };
}
