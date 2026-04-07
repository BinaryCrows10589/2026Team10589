package binarycrows.robot.SeasonCode.Autons.Data;

import binarycrows.robot.CrowMotion.UserSide.CMAutonPoint;
import edu.wpi.first.math.util.Units;

/** Format: [type_]fieldObject_sideOfFieldObject **/
public class Points {
    // in use
    public static final CMAutonPoint startPosition_DepotTrench_Wall = new CMAutonPoint(4.44, Units.inchesToMeters(317.69-4-(33.5)/2), 0, 0, 0, 0);
    // in use
    public static final CMAutonPoint startPosition_DepotTrench_Wall_Second = new CMAutonPoint(4.44, Units.inchesToMeters(317.69-20-(33.5)/2), 0, 0, 0, 0);

    // in use
    public static final CMAutonPoint startPosition_HumanPlayerTrench_Wall = new CMAutonPoint(4.44, 8.069- Units.inchesToMeters(317.69-4-(33.5)/2), 0, 0, 0, 0);
    // in use
    public static final CMAutonPoint CenterFuelQuadrant_OwnAllianceHumanPlayer = new CMAutonPoint(7.775+0, 8.069- (5.580), 0, 0, 0, 0);
    // in use
    public static final CMAutonPoint crossOverPointHumanPlayer = new CMAutonPoint(5.96, 8.069- (5.855));
    // in use
    public static final CMAutonPoint startPosition_HumanPlayerTrench_Wall_Second = new CMAutonPoint(4.44, 8.069- Units.inchesToMeters(317.69-20-(33.5)/2), 0, 0, 0, 0);

    // in use
    public static final CMAutonPoint InitialFuelQuadrant_OwnAllianceDepot = new CMAutonPoint(7.775, 7.554, 0, 0, 0, 0);
    // in use
    public static final CMAutonPoint CenterFuelQuadrant_OwnAllianceDepot = new CMAutonPoint(7.775+0, 5.580, 0, 0, 0, 0);

    // in use
    public static final CMAutonPoint crossOverPoint = new CMAutonPoint(5.96, 5.855);
    // in use
    public static final CMAutonPoint ShootPosition_DepotTrench_Wall = new CMAutonPoint(3.145, 7.554, 0, 0, 0, 0);
    // in use
    public static final CMAutonPoint MidFuelQuadrant_OwnAllianceDepot = new CMAutonPoint(7.775, 5.225, 0, 0, 0, 0);
    // in use
    public static final CMAutonPoint ShootPosition_DepotRamp_Center = new CMAutonPoint(3.5, 5.5, 0, 0, 0, 0);

}
