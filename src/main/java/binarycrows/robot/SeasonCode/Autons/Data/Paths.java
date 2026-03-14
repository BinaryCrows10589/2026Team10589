package binarycrows.robot.SeasonCode.Autons.Data;

import binarycrows.robot.SeasonCode.Autons.Utils.Path;

public class Paths {
    // Control points are determined in simulation

    public static final Path startPosition_HumanPlayerTrench_Wall_L_In =
        new Path(new Path.PathRotation[] {new Path.PathRotation(100, 0, 0.5)},
        Points.startPosition_HumanPlayerTrench_Wall,
        Points.InitialFuelQuadrant_OwnAllianceHumanPlayer,
        Points.CenterFuelQuadrant_OwnAllianceHumanPlayer);
    
    public static final Path HumanPlayerSide_L_Out_Trench =
        new Path(new Path.PathRotation[] {new Path.PathRotation(0, 0, 0.5)},
        Points.CenterFuelQuadrant_OwnAllianceHumanPlayer,
        Points.InitialFuelQuadrant_OwnAllianceHumanPlayer,
        Points.ShootPosition_HumanPlayerTrench_Wall);
    
    // Fill in points:

    public static final Path HumanPlayerSide_L_Out_Ramp =
        new Path(new Path.PathRotation[] {new Path.PathRotation(0, 0, 0.5)},
        Points.CenterFuelQuadrant_OwnAllianceHumanPlayer,
        Points.InitialFuelQuadrant_OwnAllianceHumanPlayer,
        Points.ShootPosition_HumanPlayerTrench_Wall);
    
    public static final Path startPosition_HumanPlayerTrench_Wall_P_In =
        new Path(new Path.PathRotation[] {new Path.PathRotation(0, 0, 0.5)},
        Points.CenterFuelQuadrant_OwnAllianceHumanPlayer,
        Points.InitialFuelQuadrant_OwnAllianceHumanPlayer,
        Points.ShootPosition_HumanPlayerTrench_Wall);
    
    public static final Path HumanPlayerSide_P_Out_Trench =
        new Path(new Path.PathRotation[] {new Path.PathRotation(0, 0, 0.5)},
        Points.CenterFuelQuadrant_OwnAllianceHumanPlayer,
        Points.InitialFuelQuadrant_OwnAllianceHumanPlayer,
        Points.ShootPosition_HumanPlayerTrench_Wall);

    public static final Path HumanPlayerSide_P_Out_Ramp =
        new Path(new Path.PathRotation[] {new Path.PathRotation(0, 0, 0.5)},
        Points.CenterFuelQuadrant_OwnAllianceHumanPlayer,
        Points.InitialFuelQuadrant_OwnAllianceHumanPlayer,
        Points.ShootPosition_HumanPlayerTrench_Wall);
}
