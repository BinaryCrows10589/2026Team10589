package binarycrows.robot.SeasonCode.Autons;

import binarycrows.robot.StateRequest;
import binarycrows.robot.Enums.StateRequestGroupChildTimeoutBehavior;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.SeasonCode.Autons.Data.Points;
import binarycrows.robot.SeasonCode.SubStateManagers.Shooting.ShootingStateRequest;
import binarycrows.robot.StateRequestGroup.SequentialGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class StartPosition_HumanPlayerRamp_Trench_ShootPreloads {
    public static Pose2d startingPoint = new Pose2d(Points.startPosition_HumanPlayerRamp_Trench.getTranslation2d(), Rotation2d.fromDegrees(-90));

    public static SequentialGroup getAutonomous() {
        return new SequentialGroup(
            StateRequestPriority.NORMAL,
            15*1000,
            new StateRequest<>(ShootingStateRequest.SHOOT_PRELOADS, StateRequestPriority.NORMAL, 3*1000, StateRequestGroupChildTimeoutBehavior.SKIP)
        );

        };
}
