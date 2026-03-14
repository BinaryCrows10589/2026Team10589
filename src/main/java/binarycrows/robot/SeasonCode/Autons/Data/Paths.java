package binarycrows.robot.SeasonCode.Autons.Data;

import org.opencv.core.Point;

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

    public static final Path startPosition_HumanPlayerTrench_Wall_L_In =
        new Path(new Path.PathRotation[] {new Path.PathRotation(100, 0, 0.5)},
        Points.startPosition_HumanPlayerTrench_Wall,
        Points.InitialFuelQuadrant_OwnAllianceHumanPlayer,
        Points.CenterFuelQuadrant_OwnAllianceHumanPlayer);
    
    public static final Path startPosition_HumanPlayerTrench_Wall_L_Out =
        new Path(new Path.PathRotation[] {new Path.PathRotation(0, 0, 0.5)},
        Points.CenterFuelQuadrant_OwnAllianceHumanPlayer,
        Points.InitialFuelQuadrant_OwnAllianceHumanPlayer,
        Points.ShootPosition_HumanPlayerTrench_Wall);
}
