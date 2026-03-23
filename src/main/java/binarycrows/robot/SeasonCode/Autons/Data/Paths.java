package binarycrows.robot.SeasonCode.Autons.Data;

import binarycrows.robot.CrowMotion.UserSide.CMAutonPoint;
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
    
    public static final Path HumanPlayerSide_L_Out_Ramp =
        new Path(new Path.PathRotation[] {new Path.PathRotation(0, 0, 0.5)},
        Points.CenterFuelQuadrant_OwnAllianceHumanPlayer,
        Points.MidFuelQuadrant_OwnAllianceHumanPlayer,
        Points.ShootPosition_HumanPlayerRamp_Center);
    
    public static final Path startPosition_HumanPlayerTrench_Wall_P_In =
        new Path(new Path.PathRotation[] {new Path.PathRotation(0, 0, 0.5)},
        Points.startPosition_HumanPlayerTrench_Wall,
        Points.InitialFuelQuadrant_OwnAllianceHumanPlayer,
        Points.HumanPlayer_P_In_ControlPoints[0],
        Points.HumanPlayer_P_In_ControlPoints[1],
        Points.HumanPlayer_P_In_ControlPoints[2],
        Points.PEnd_OwnAllianceHumanPlayer);
    
    public static final Path HumanPlayerSide_P_Out_Trench =
        new Path(new Path.PathRotation[] {new Path.PathRotation(0, 0, 0.5)},
        Points.PEnd_OwnAllianceHumanPlayer,
        new CMAutonPoint(6, 0.5),
        Points.ShootPosition_HumanPlayerTrench_Wall);

    public static final Path HumanPlayerSide_P_Out_Ramp =
        new Path(new Path.PathRotation[] {new Path.PathRotation(0, 0, 0.5)},
        Points.PEnd_OwnAllianceHumanPlayer,
        new CMAutonPoint(6, 2.5),
        Points.ShootPosition_HumanPlayerRamp_Center);

    public static final Path HumanPlayerSide_P_In_Ramp =
        new Path(new Path.PathRotation[] {new Path.PathRotation(0, 0, 0.5)},
        Points.ShootPosition_HumanPlayerRamp_Center,
        Points.ShootPosition_HumanPlayerRamp_Center_to_InitialFuelQuadrant_OwnAllianceHumanPlayer_ControlPoint,
        Points.InitialFuelQuadrant_OwnAllianceHumanPlayer,
        Points.HumanPlayer_P_In_ControlPoints[0],
        Points.HumanPlayer_P_In_ControlPoints[1],
        Points.HumanPlayer_P_In_ControlPoints[2],
        Points.PEnd_OwnAllianceHumanPlayer);
    
    public static final Path HumanPlayerSide_L_In_Ramp =
        new Path(new Path.PathRotation[] {new Path.PathRotation(0, 0, 0.5)},
        Points.ShootPosition_HumanPlayerRamp_Center,
        Points.ShootPosition_HumanPlayerRamp_Center_to_InitialFuelQuadrant_OwnAllianceHumanPlayer_ControlPoint,
        Points.InitialFuelQuadrant_OwnAllianceHumanPlayer,
        Points.CenterFuelQuadrant_OwnAllianceHumanPlayer);
    
    // Depot side
    
    public static final Path startPosition_DepotTrench_Wall_L_In =
        new Path(new Path.PathRotation[] {new Path.PathRotation(100, 0, 1)},
        Points.startPosition_DepotTrench_Wall,
        new CMAutonPoint(7.775+0.5, 7.554+0.5, 0, 0, 0, 0),
        Points.CenterFuelQuadrant_OwnAllianceDepot);
    
    public static final Path startPosition_DepotTrench_Wall_L_Arch =
        new Path(new Path.PathRotation[] {new Path.PathRotation(0, 0, 0.5)},
        Points.CenterFuelQuadrant_OwnAllianceDepot,
        new CMAutonPoint(7.690, 3.371-2),
        new CMAutonPoint(5.714, 6.455-.5));

    public static final Path startPosition_DepotTrench_Wall_L_Return =
        new Path(new Path.PathRotation[] {new Path.PathRotation(0, 0, 0.5)},
        new CMAutonPoint(5.714, 6.455-.5), // TODO: Make constant
        new CMAutonPoint(4.598, 5.639),
        new CMAutonPoint(2.738, 6.043));
    
    public static final Path DepotSide_L_Out_Trench =
        new Path(new Path.PathRotation[] {new Path.PathRotation(0, 0, 1)},
        Points.CenterFuelQuadrant_OwnAllianceDepot,
        Points.InitialFuelQuadrant_OwnAllianceDepot,
        Points.ShootPosition_DepotTrench_Wall);
    
    public static final Path DepotSide_L_Out_Ramp =
        new Path(new Path.PathRotation[] {new Path.PathRotation(0, 0, 0.5)},
        Points.CenterFuelQuadrant_OwnAllianceDepot,
        Points.MidFuelQuadrant_OwnAllianceDepot,
        Points.ShootPosition_DepotRamp_Center);
    
    public static final Path startPosition_DepotTrench_Wall_P_In =
        new Path(new Path.PathRotation[] {new Path.PathRotation(0, 0, 0.5)},
        Points.startPosition_DepotTrench_Wall,
        Points.InitialFuelQuadrant_OwnAllianceDepot);
    
    public static final Path startPosition_DepotTrench_Wall_P_Arc =
        new Path(new Path.PathRotation[] {new Path.PathRotation(0, 0, 0.5)},
        Points.InitialFuelQuadrant_OwnAllianceDepot,
        Points.PEnd_OwnAllianceDepot);
    
    public static final Path DepotSide_P_Out_Trench =
        new Path(new Path.PathRotation[] {new Path.PathRotation(0, 0, 0.5)},
        Points.PEnd_OwnAllianceDepot,
        new CMAutonPoint(6, 7.5),
        Points.ShootPosition_DepotTrench_Wall);

    public static final Path DepotSide_P_Out_Ramp =
        new Path(new Path.PathRotation[] {new Path.PathRotation(0, 0, 0.5)},
        Points.PEnd_OwnAllianceDepot,
        new CMAutonPoint(6, 5.5),
        Points.ShootPosition_DepotTrench_Wall);

    public static final Path DepotSide_P_In_Ramp =
        new Path(new Path.PathRotation[] {new Path.PathRotation(0, 0, 0.5)},
        Points.ShootPosition_DepotTrench_Wall,
        Points.ShootPosition_DepotRamp_Center_to_InitialFuelQuadrant_OwnAllianceDepot_ControlPoint,
        Points.InitialFuelQuadrant_OwnAllianceDepot,
        Points.PEnd_OwnAllianceDepot);
    
    public static final Path DepotSide_L_In_Ramp =
        new Path(new Path.PathRotation[] {new Path.PathRotation(0, 0, 0.5)},
        Points.ShootPosition_DepotRamp_Center,
        Points.ShootPosition_DepotRamp_Center_to_InitialFuelQuadrant_OwnAllianceDepot_ControlPoint,
        Points.InitialFuelQuadrant_OwnAllianceDepot,
        Points.CenterFuelQuadrant_OwnAllianceDepot);

    // end
    
    public static final Path DepotSide_Ramp_to_Depot =
        new Path(new Path.PathRotation[] {new Path.PathRotation(0, 0, 0.5)},
        Points.ShootPosition_DepotRamp_Center,
        new CMAutonPoint(2.15, 6.5),
        Points.DepotIntakingPosition);
    
    public static final Path DepotSide_Trench_to_Depot =
        new Path(new Path.PathRotation[] {new Path.PathRotation(0, 0, 0.5)},
        Points.ShootPosition_DepotTrench_Wall,
        new CMAutonPoint(2.15, 5.4),
        Points.DepotIntakingPosition);
    
    public static final Path DepotSide_Depot_to_Ramp =
        new Path(new Path.PathRotation[] {new Path.PathRotation(0, 0, 0.5)},
        Points.DepotIntakingPosition,
        new CMAutonPoint(2.15, 6.5),
        Points.ShootPosition_DepotRamp_Center);

    public static final Path DepotSide_Depot_to_Trench =
        new Path(new Path.PathRotation[] {new Path.PathRotation(0, 0, 0.5)},
        Points.DepotIntakingPosition,
        new CMAutonPoint(2.15, 5.4),
        Points.ShootPosition_DepotTrench_Wall);
}
