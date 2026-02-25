package binarycrows.robot.SeasonCode.Autons.Data;

import binarycrows.robot.CrowMotion.UserSide.CMAutonPoint;
import binarycrows.robot.CrowMotion.UserSide.CMRotation;
import binarycrows.robot.SeasonCode.Autons.Utils.Path;
import binarycrows.robot.SeasonCode.Autons.Utils.Path.PathRotation;

public class Paths {
    // Control points are determined in simulation

    public static final Path startPosition_HumanPlayerRamp_Trench_to_InitialFuelQuadrant_OwnAllianceHumanPlayer = 
        new Path(new Path.PathRotation[] {new Path.PathRotation(90, 0, 1)},
        Points.startPosition_HumanPlayerRamp_Trench, 
        Points.InitialFuelQuadrant_OwnAllianceHumanPlayer);
}
