package binarycrows.robot.SeasonCode.Constants;

import binarycrows.robot.Utils.ConversionUtils;
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

    public static final double maxTurretDeltaRad = Units.degreesToRadians(12);
    public static final double maxHoodDeltaRad = Units.degreesToRadians(8);
    public static final double maxFlywheelDelta = 550;
    public static final double maxVelocity = 4.4;
    public static final double maxAcceleration = 4.4;
    public static final double maxJerk = 10;
    public static final double maxVelocityLarge = 4.4;
    public static final double maxTurretX = 5.631;
    public static final double maxDistanceFromGoal = 5;
    public static final double maxDistanceFromGoalLarge = 6;

    // If we are within any of these bounds, the hood will retract automatically
    public static final Rectangle2d trenchBoundsHumanPlayerOwnSide = new Rectangle2d(new Translation2d(4, 1.3), new Translation2d(5.25, 0));
    public static final Rectangle2d trenchBoundsDepotOwnSide = new Rectangle2d(new Translation2d(4, 8.1), new Translation2d(5.25, 6.8));
    public static final Rectangle2d trenchBoundsHumanPlayerOppositeSide = new Rectangle2d(new Translation2d(11.3, 1.3), new Translation2d(12.5, 0));
    public static final Rectangle2d trenchBoundsDepotOppositeSide = new Rectangle2d(new Translation2d(11.3, 8.1), new Translation2d(12.5, 6.8));

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

        new double[] {1.323, 0, 52.5, .83},
        new double[] {2.176, .125, 50, .93},
        new double[] {3.027, .19, 50, .94},
        new double[] {3.572, .225, 52, .91},
        new double[] {4.543, .325, 56.5, .93},
        new double[] {5.487, .35, 60, 1},
        new double[] {6.556, .375, 66, 1.11},

    }, 
    false);
}
