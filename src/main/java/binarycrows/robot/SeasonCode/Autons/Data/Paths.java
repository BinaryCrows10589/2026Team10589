package binarycrows.robot.SeasonCode.Autons.Data;

import binarycrows.robot.CrowMotion.UserSide.CMAutonPoint;
import binarycrows.robot.SeasonCode.Autons.Utils.Path;

public class Paths {
    // Control points are determined in simulation

    // INVERT
    public static final Path startPosition_HumanPlayerTrench_Wall_L_In =
        new Path(
        Points.startPosition_HumanPlayerTrench_Wall,
        new CMAutonPoint(8.575, 8.069- (7.554), 0, 0, 0, 0),
        new CMAutonPoint(8.575, 8.069- (6.804), 0, 0, 0, 0),
        Points.CenterFuelQuadrant_OwnAllianceHumanPlayer);
    
    public static final Path startPosition_HumanPlayerTrench_Wall_L_Arch_Half_One =
        new Path(
        Points.CenterFuelQuadrant_OwnAllianceHumanPlayer,
        new CMAutonPoint(6.44, 8.069- (3.071)),
        Points.crossOverPointHumanPlayer);

    public static final Path startPosition_HumanPlayerTrench_Wall_L_Return =
        new Path(
        Points.crossOverPointHumanPlayer, 
        new CMAutonPoint(4.598, 8.069- (5.139)),

        new CMAutonPoint(2.738, 8.069- 6.043));

    public static final Path startPosition_HumanPlayerTrench_Wall_L_In_Second_Crawl = new Path(
        new CMAutonPoint(2.738, 8.069- 6.043),
        new CMAutonPoint(2.738, 8.069- (7.557))
    );

    public static final Path startPosition_HumanPlayerTrench_Wall_L_In_Second = new Path(
        new CMAutonPoint(2.738, 8.069- (7.557)),
        Points.startPosition_HumanPlayerTrench_Wall_Second,
        new CMAutonPoint(7.25, 8.069- (7.754), 0, 0, 0, 0),
        new CMAutonPoint(6, 8.069- 6.603, 0, 0, 0, 0),
        new CMAutonPoint(6, 8.069-4)
    );

    public static final Path startPosition_HumanPlayerTrench_Wall_L_Arch_Half_One_Second =
        new Path(
        Points.crossOverPointHumanPlayer,
        new CMAutonPoint(6, 8.069- 4, 0, 0, 0, 0),
        new CMAutonPoint(6, 8.069- 6.603, 0, 0, 0, 0),

        Points.crossOverPointHumanPlayer);

    public static final Path startPosition_HumanPlayerTrench_Wall_L_Second_Crawl = new Path(
        new CMAutonPoint(2.719, 8.069- 5.698),
        new CMAutonPoint(2.738, 8.069- (7.707))
    );
    
    // Depot side
    
    public static final Path startPosition_DepotTrench_Wall_L_In =
        new Path(
        Points.startPosition_DepotTrench_Wall,
        new CMAutonPoint(8.575, 7.554, 0, 0, 0, 0),
        new CMAutonPoint(8.575, 6.804, 0, 0, 0, 0),
        Points.CenterFuelQuadrant_OwnAllianceDepot);
    
    public static final Path startPosition_DepotTrench_Wall_L_Arch_Half_One =
        new Path(
        Points.CenterFuelQuadrant_OwnAllianceDepot,
        new CMAutonPoint(6.44, 3.071),
        Points.crossOverPoint);

    public static final Path startPosition_DepotTrench_Wall_L_Arch_Half_One_Second =
        new Path(
        Points.crossOverPoint,
        new CMAutonPoint(6, 4),
        new CMAutonPoint(6, 6.603, 0, 0, 0, 0),
        Points.crossOverPoint);
    
    public static final Path startPosition_DepotTrench_Wall_L_Return =
        new Path(
        Points.crossOverPoint,
        new CMAutonPoint(4.598, 5.139),
        new CMAutonPoint(2.738, 6.043));

    public static final Path startPosition_DepotTrench_Wall_L_In_Second_Crawl = new Path(
        new CMAutonPoint(2.738, 6.043),
        new CMAutonPoint(2.738, 7.557)
    );

    public static final Path startPosition_DepotTrench_Wall_L_In_Second = new Path(
        new CMAutonPoint(2.738, 7.557),
        Points.startPosition_DepotTrench_Wall_Second,
        new CMAutonPoint(7.25, 7.754, 0, 0, 0, 0),
        new CMAutonPoint(6, 6.603, 0, 0, 0, 0),
        new CMAutonPoint(6, 4)
    );

    public static final Path startPosition_DepotTrench_Wall_L_Second_Crawl = new Path(
        new CMAutonPoint(2.719, 5.698),
        new CMAutonPoint(2.738, 7.707)
    );
}
