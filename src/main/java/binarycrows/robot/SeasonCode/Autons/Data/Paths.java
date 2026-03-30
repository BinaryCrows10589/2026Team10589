package binarycrows.robot.SeasonCode.Autons.Data;

import binarycrows.robot.CrowMotion.UserSide.CMAutonPoint;
import binarycrows.robot.SeasonCode.Autons.Utils.Path;

public class Paths {
    // Control points are determined in simulation

    // INVERT
    public static final Path startPosition_HumanPlayerTrench_Wall_L_In =
        new Path(
        Points.startPosition_HumanPlayerTrench_Wall,
        new CMAutonPoint(7.775+0.8, 8.069- (7.554-0), 0, 0, 0, 0),
        new CMAutonPoint(7.775+.8, 8.069- (7.554-0.75), 0, 0, 0, 0),
        Points.CenterFuelQuadrant_OwnAllianceHumanPlayer);
    
    public static final Path startPosition_HumanPlayerTrench_Wall_L_Arch_Half_One =
        new Path(
        Points.CenterFuelQuadrant_OwnAllianceHumanPlayer,
        new CMAutonPoint(7.690-1.25, 8.069- (3.371-0.3)),
        Points.crossOverPointHumanPlayer);

    public static final Path startPosition_HumanPlayerTrench_Wall_L_Return =
        new Path(
        Points.crossOverPointHumanPlayer, // TODO: Make constant
        new CMAutonPoint(4.598, 8.069- (5.639-0.5)),

        new CMAutonPoint(2.738, 8.069- 6.043));

    public static final Path startPosition_HumanPlayerTrench_Wall_L_In_Second_Crawl = new Path(
        new CMAutonPoint(2.738, 8.069- 6.043),
        new CMAutonPoint(2.738, 8.069- (7.457+0.25))
    );

    public static final Path startPosition_HumanPlayerTrench_Wall_L_In_Second = new Path(
        new CMAutonPoint(2.738, 8.069- (7.457+0.25)),
        Points.startPosition_HumanPlayerTrench_Wall_Second,
        new CMAutonPoint(7.775+.8, 8.069- (7.554-0), 0, 0, 0, 0),
        new CMAutonPoint(7.817, 8.069- 6.603, 0, 0, 0, 0),
        Points.CenterFuelQuadrant_OwnAllianceHumanPlayer_Second
    );

    public static final Path startPosition_HumanPlayerTrench_Wall_L_Arch_Half_One_Second =
        new Path(
        Points.CenterFuelQuadrant_OwnAllianceHumanPlayer_Second,
        new CMAutonPoint(7.817, 8.069- 6.603, 0, 0, 0, 0),
        Points.crossOverPointHumanPlayer);

    public static final Path startPosition_HumanPlayerTrench_Wall_L_Second_Crawl = new Path(
        new CMAutonPoint(2.719, 8.069- 5.698),
        new CMAutonPoint(2.738, 8.069- (7.457+0.25))
    );
    
    // Depot side
    
    public static final Path startPosition_DepotTrench_Wall_L_In =
        new Path(
        Points.startPosition_DepotTrench_Wall,
        new CMAutonPoint(7.775+0.8, 7.554-0, 0, 0, 0, 0),
        new CMAutonPoint(7.775+.8, 7.554-0.75, 0, 0, 0, 0),
        Points.CenterFuelQuadrant_OwnAllianceDepot);
    
    public static final Path startPosition_DepotTrench_Wall_L_Arch_Half_One =
        new Path(
        Points.CenterFuelQuadrant_OwnAllianceDepot,
        new CMAutonPoint(7.690-1.25, 3.371-0.3),
        Points.crossOverPoint);

    public static final Path startPosition_DepotTrench_Wall_L_Arch_Half_One_Second =
        new Path(
        Points.CenterFuelQuadrant_OwnAllianceDepot_Second,
        new CMAutonPoint(7.817, 6.603, 0, 0, 0, 0),
        Points.crossOverPoint);
    
  

    public static final Path startPosition_DepotTrench_Wall_L_Return =
        new Path(
        Points.crossOverPoint, // TODO: Make constant
        new CMAutonPoint(4.598, 5.639-0.5),

        new CMAutonPoint(2.738, 6.043));

    public static final Path startPosition_DepotTrench_Wall_L_In_Second_Crawl = new Path(
        new CMAutonPoint(2.738, 6.043),
        new CMAutonPoint(2.738, 7.457+0.25)
    );
    public static final Path startPosition_DepotTrench_Wall_L_In_Second = new Path(
        new CMAutonPoint(2.738, 7.457+0.25),
        Points.startPosition_DepotTrench_Wall_Second,
        new CMAutonPoint(7.775+.8, 7.554-0, 0, 0, 0, 0),
        new CMAutonPoint(7.817, 6.603, 0, 0, 0, 0),
        Points.CenterFuelQuadrant_OwnAllianceDepot_Second
    );
    public static final Path startPosition_DepotTrench_Wall_L_Insertion = new Path(
        new CMAutonPoint(5.825, 7.418),
        new CMAutonPoint(5.721, 4.598)
    );
    public static final Path startPosition_DepotTrench_Wall_L_Retraction = new Path(
        new CMAutonPoint(5.721, 4.598),
        new CMAutonPoint(5.721, 5.128+0.5)
    );
    public static final Path startPosition_DepotTrench_Wall_L_Return_Second = new Path(
        new CMAutonPoint(5.721, 5.128+0.5),
        new CMAutonPoint(2.719, 5.698)
    );
    public static final Path startPosition_DepotTrench_Wall_L_Second_Crawl = new Path(
        new CMAutonPoint(2.719, 5.698),
        new CMAutonPoint(2.738, 7.457+0.25)
    );
    
    public static final Path DepotSide_L_Out_Trench =
        new Path(
        Points.CenterFuelQuadrant_OwnAllianceDepot,
        Points.InitialFuelQuadrant_OwnAllianceDepot,
        Points.ShootPosition_DepotTrench_Wall);
    
    public static final Path DepotSide_L_Out_Ramp =
        new Path(
        Points.CenterFuelQuadrant_OwnAllianceDepot,
        Points.MidFuelQuadrant_OwnAllianceDepot,
        Points.ShootPosition_DepotRamp_Center);
    
    public static final Path startPosition_DepotTrench_Wall_P_In =
        new Path(
        Points.startPosition_DepotTrench_Wall,
        Points.InitialFuelQuadrant_OwnAllianceDepot);
    
    public static final Path startPosition_DepotTrench_Wall_P_Arc =
        new Path(
        Points.InitialFuelQuadrant_OwnAllianceDepot,
        Points.PEnd_OwnAllianceDepot);
    
    public static final Path DepotSide_P_Out_Trench =
        new Path(
        Points.PEnd_OwnAllianceDepot,
        new CMAutonPoint(6, 7.5),
        Points.ShootPosition_DepotTrench_Wall);

    public static final Path DepotSide_P_Out_Ramp =
        new Path(
        Points.PEnd_OwnAllianceDepot,
        new CMAutonPoint(6, 5.5),
        Points.ShootPosition_DepotTrench_Wall);

    public static final Path DepotSide_P_In_Ramp =
        new Path(
        Points.ShootPosition_DepotTrench_Wall,
        Points.ShootPosition_DepotRamp_Center_to_InitialFuelQuadrant_OwnAllianceDepot_ControlPoint,
        Points.InitialFuelQuadrant_OwnAllianceDepot,
        Points.PEnd_OwnAllianceDepot);
    
    public static final Path DepotSide_L_In_Ramp =
        new Path(
        Points.ShootPosition_DepotRamp_Center,
        Points.ShootPosition_DepotRamp_Center_to_InitialFuelQuadrant_OwnAllianceDepot_ControlPoint,
        Points.InitialFuelQuadrant_OwnAllianceDepot,
        Points.CenterFuelQuadrant_OwnAllianceDepot);

    // end
    
    public static final Path DepotSide_Ramp_to_Depot =
        new Path(
        Points.ShootPosition_DepotRamp_Center,
        new CMAutonPoint(2.15, 6.5),
        Points.DepotIntakingPosition);
    
    public static final Path DepotSide_Trench_to_Depot =
        new Path(
        Points.ShootPosition_DepotTrench_Wall,
        new CMAutonPoint(2.15, 5.4),
        Points.DepotIntakingPosition);
    
    public static final Path DepotSide_Depot_to_Ramp =
        new Path(
        Points.DepotIntakingPosition,
        new CMAutonPoint(2.15, 6.5),
        Points.ShootPosition_DepotRamp_Center);

    public static final Path DepotSide_Depot_to_Trench =
        new Path(
        Points.DepotIntakingPosition,
        new CMAutonPoint(2.15, 5.4),
        Points.ShootPosition_DepotTrench_Wall);
}
