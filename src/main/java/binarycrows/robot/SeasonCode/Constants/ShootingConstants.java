package binarycrows.robot.SeasonCode.Constants;

import binarycrows.robot.Utils.ConversionUtils;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ShootingConstants {
    public static final Transform2d robotToTurret = new Transform2d(new Translation2d(0, 0), Rotation2d.kZero); //TO-DO: Set to correct translation
    public static final int numberOfAlgorithmIterations = 20; //TO-DO: This can certainly be much lower...
    public static final double dragCoefficient = 0.2; //TO-DO: tune... ...a lot

    public static final double maxTurretDeltaRad = 10;
    public static final double maxHoodDeltaRad = 10;
    public static final double maxFlywheelDelta = 10;
    public static final double maxVelocity = 4.4;
    public static final double maxAcceleration = 4.4;
    public static final double maxJerk = 4.4;
    public static final double maxVelocityLarge = 4.4;
    public static final double maxTurretX = ConversionUtils.inchesToMeters(182.11);
    public static final double maxDistanceFromGoal = 5;
    public static final double maxDistanceFromGoalLarge = 6;

    // If we are within any of these bounds, the hood will retract automatically
    public static final Rectangle2d trenchBoundsHumanPlayerOwnSide = new Rectangle2d(new Translation2d(4, 1.3), new Translation2d(5.25, 0));
    public static final Rectangle2d trenchBoundsDepotOwnSide = new Rectangle2d(new Translation2d(4, 8.1), new Translation2d(5.25, 6.8));
    public static final Rectangle2d trenchBoundsHumanPlayerOppositeSide = new Rectangle2d(new Translation2d(11.3, 1.3), new Translation2d(12.5, 0));
    public static final Rectangle2d trenchBoundsDepotOppositeSide = new Rectangle2d(new Translation2d(11.3, 8.1), new Translation2d(12.5, 6.8));
}
