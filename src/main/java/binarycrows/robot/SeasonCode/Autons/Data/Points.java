package binarycrows.robot.SeasonCode.Autons.Data;

import binarycrows.robot.CrowMotion.UserSide.CMAutonPoint;

/** Format: [type_]fieldObject_sideOfFieldObject **/
public class Points {
    public static final CMAutonPoint startPosition_HumanPlayerRamp_Trench = new CMAutonPoint(3.6, 2.05, 0, 0, 0, 0);
    public static final CMAutonPoint startPosition_HumanPlayerTrench_Wall = new CMAutonPoint(4.44, 0.496, 0, 0, 0, 0);
    public static final CMAutonPoint InitialFuelQuadrant_OwnAllianceHumanPlayer = new CMAutonPoint(7.775, 0.496, 0, 0, 0, 0);
    public static final CMAutonPoint CenterFuelQuadrant_OwnAllianceHumanPlayer = new CMAutonPoint(7.775, 3.457, 0, 0, 0, 0);
    public static final CMAutonPoint ShootPosition_HumanPlayerTrench_Wall = new CMAutonPoint(3.145, 0.496, 0, 0, 0, 0);
    public static final CMAutonPoint MidFuelQuadrant_OwnAllianceHumanPlayer = new CMAutonPoint(7.775, 2.775, 0, 0, 0, 0);
    public static final CMAutonPoint ShootPosition_HumanPlayerRamp_Center = new CMAutonPoint(3.5, 2.5, 0, 0, 0, 0);
    public static final CMAutonPoint PEnd_OwnAllianceHumanPlayer = new CMAutonPoint(7.280, 2.175, 0, 0, 0, 0);
    public static final CMAutonPoint startPosition_DepotTrench_Wall = new CMAutonPoint(4.44, 7.554, 0, 0, 0, 0);
    public static final CMAutonPoint InitialFuelQuadrant_OwnAllianceDepot = new CMAutonPoint(7.775, 7.554, 0, 0, 0, 0);
    public static final CMAutonPoint CenterFuelQuadrant_OwnAllianceDepot = new CMAutonPoint(7.775, 4.580, 0, 0, 0, 0);
    public static final CMAutonPoint ShootPosition_DepotTrench_Wall = new CMAutonPoint(3.145, 7.554, 0, 0, 0, 0);
    public static final CMAutonPoint MidFuelQuadrant_OwnAllianceDepot = new CMAutonPoint(7.775, 5.225, 0, 0, 0, 0);
    public static final CMAutonPoint ShootPosition_DepotRamp_Center = new CMAutonPoint(3.5, 5.5, 0, 0, 0, 0);
    public static final CMAutonPoint PEnd_OwnAllianceDepot = new CMAutonPoint(7.280, 5.825, 0, 0, 0, 0);
    public static final CMAutonPoint DepotIntakingPosition = new CMAutonPoint(0.472, 5.967, 0, 0, 0, 0);

    public static final CMAutonPoint[] HumanPlayer_P_In_ControlPoints = new CMAutonPoint[] {
        new CMAutonPoint(7.775, 3.2175, 0, 0, 0, 0),
        new CMAutonPoint(7.5275, 3.575, 0, 0, 0, 0),
        new CMAutonPoint(7.280, 3.2175, 0, 0, 0, 0)
    };
    public static final CMAutonPoint[] Depot_P_In_ControlPoints = new CMAutonPoint[] {
        new CMAutonPoint(7.775, 4.7825, 0, 0, 0, 0),
        new CMAutonPoint(7.5275, 4.425, 0, 0, 0, 0),
        new CMAutonPoint(7.280, 4.7825, 0, 0, 0, 0)
    };

    public static final CMAutonPoint ShootPosition_HumanPlayerRamp_Center_to_InitialFuelQuadrant_OwnAllianceHumanPlayer_ControlPoint = new CMAutonPoint(6.5, 1.8);
    public static final CMAutonPoint ShootPosition_DepotRamp_Center_to_InitialFuelQuadrant_OwnAllianceDepot_ControlPoint = new CMAutonPoint(6.5, 6.2);

}
