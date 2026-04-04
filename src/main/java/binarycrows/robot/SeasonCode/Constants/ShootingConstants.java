package binarycrows.robot.SeasonCode.Constants;

import binarycrows.robot.Utils.UnkeyedLerpTable;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class ShootingConstants {
    public static final Transform2d robotToTurret = new Transform2d(new Translation2d(Units.inchesToMeters(6.5), 0), Rotation2d.kZero);
    public static final int numberOfAlgorithmIterations = 20;
    public static final double dragCoefficient = 0;

    public static final double maxTurretDeltaRad = Units.degreesToRadians(10);
    public static final double maxHoodDeltaRad = Units.degreesToRadians(6);
    public static final double maxFlywheelDelta = 500;
    public static final double maxVelocity = 4.4;
    public static final double maxAcceleration = 4.4;
    public static final double maxJerk = 10;
    public static final double maxVelocityLarge = 4.4;
    public static final double maxTurretX = 5.631;
    public static final double maxTurretXStrict = 5.631-1;
    public static final double maxDistanceFromGoal = 5;
    public static final double maxDistanceFromGoalLarge = 6;

    // If we are within any of these bounds, the hood will retract automatically
    public static final Rectangle2d trenchBoundsHumanPlayerOwnSide = new Rectangle2d(new Translation2d(4-.3, 1.3), new Translation2d(5.25+.3, 0));
    public static final Rectangle2d trenchBoundsDepotOwnSide = new Rectangle2d(new Translation2d(4-.3, 8.1), new Translation2d(5.25+.3, 6.8));
    public static final Rectangle2d trenchBoundsHumanPlayerOppositeSide = new Rectangle2d(new Translation2d(11.3-.3, 1.3), new Translation2d(12.5+.3, 0));
    public static final Rectangle2d trenchBoundsDepotOppositeSide = new Rectangle2d(new Translation2d(11.3-.3, 8.1), new Translation2d(12.5+.3, 6.8));

    // Base table goes distance, hood angle, flywheel RPS, time of flight
    public static final UnkeyedLerpTable baseTable = new UnkeyedLerpTable(new double[][] {
        //           |distance|hood angle|flywheel|ToF|   
        //new double[] {0,       0,         0,       0}, 
        //new double[] {1.614,   0,         55,      .71}, // Will crash if there are not at least 2 values
        //new double[] {2.613,   0.165,     55,      .99},
        //new double[] {3.669,   0.25,     55,      1.08},
        //new double[] {4.901,   0.38,     57,      1}
        
        
        /*new double[] {1.561, .1, 48, .71},
        new double[] {2.452, .15, 48, 0.86},
        new double[] {3.469, .3, 48, 0.86},
        new double[] {5.159, .4, 59, 1.04}*/
        
        /*new double[] {1.15,  0,     52, .65},
        new double[] {1.555, 0.05,  52, .86},
        new double[] {2.057, 0.125, 52, .88},
        new double[] {2.456, 0.175, 52, .91},
        new double[] {2.916, 0.215, 52, 1.01},
        new double[] {3.322, 0.24,  52, 1.06},
        new double[] {3.855, .3,    55, 1.03},
        new double[] {4.364, .35,   56, 1.03},
        new double[] {4.844, 0.37,  60, 1.14}*/
        // distence, hood angle, flywheel, tof
        
        /*new double[] {1.323, 0,    52.5+3.5, .83},
        new double[] {2.176, .125, 50  +3.5, .93},
        new double[] {3.027, .19,  50  +3.5, .94},
        new double[] {3.572, .225, 52  +3.5, .91},
        new double[] {4.543, .325, 56.5+3.5, .93},
        new double[] {5.487, .35,  60  +3.5, 1},
        new double[] {6.556, .375, 66  +3.5, 1.11}, */

        // distance hood angle, flywheel, tof
        new double[] {1.400, 0, 63.75, 1.2-.44},
        new double[] {1.916, 0, 65, 2.78-1.86},
        new double[] {2.314, .08, 65, 10.8-9.81},
        new double[] {2.851, .12, 65, 3.56-2.36},
        new double[] {3.081, .145, 65, 15.06-13.85},
        new double[] {3.282, .16, 65, 15.39-14.16},
        new double[] {3.457, .18, 65, 4.44-3.15},
        new double[] {3.625, .195, 65, 3.94-2.62},
        new double[] {3.926, .205, 65, 4.15-2.79},
        new double[] {4.186, .21, 65, 2.24-.93},
        new double[] {4.437, .22, 65, 2.74-1.42},
        new double[] {4.702, .235, 65, 4.09-2.72},
        new double[] {5.224, .26, 65, 3.73-2.38},
        new double[] {5.782, .32, 65, 10.98-9.58},
        new double[] {6.819, .4, 70, 3.37-1.91},
        new double[] {7.531, .45, 73, 6.5-5.05},
        new double[] {8.396, .45, 80, 2.93-1.53},
        new double[] {8.4, .45, 100, 3},
        new double[] {11.012, .45, 100, 4}

    }, 
    false);
    public static final double positionFudgeFactorIncrement = Units.inchesToMeters(3.0);
}
