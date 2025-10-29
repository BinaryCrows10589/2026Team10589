package binarycrows.robot.SeasonCode.Constants;

import binarycrows.robot.Utils.ConversionUtils;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class SwerveDriveConstants {
    public static final String CANLoopName = "*";

    public static final String frontLeftModuleName = "FrontLeftModule";
    public static final String frontRightModuleName = "FrontRightModule";
    public static final String backLeftModuleName = "BackLeftModule";
    public static final String backRightModuleName = "BackRightModule";

    public static final boolean frontLeftDriveInverted = false;
    public static final boolean frontRightDriveInverted = false;
    public static final boolean backLeftDriveInverted = false;
    public static final boolean backRightDriveInverted = false;

    public static final boolean frontLeftTurnInverted = true;
    public static final boolean frontRightTurnInverted = true;
    public static final boolean backLeftTurnInverted = true;
    public static final boolean backRightTurnInverted = true;

    public static final double frontLeftTurnEncoderOffset = 0.300049;
    public static final double frontRightTurnEncoderOffset = 0.368164;
    public static final double backLeftTurnEncoderOffset = 0.186768;
    public static final double backRightTurnEncoderOffset = 0.154053;

    public static final int maxDriveMotorVoltage = 13;
    public static final int maxTurnMotorVoltage = 2;

    public static final double maxSpeedMPS = 4.317;

    public static final double turnPIDValueP = .24472;
    public static final double turnPIDValueI = 0;
    public static final double turnPIDValueD = 0;
    public static final double turnPIDValueFF = 0;
    public static final double turnPIDValueIZone = 0.5 / 360; // 1/2 degrees converted to rotations

    public static final double turnGearRatio = 12.1;

    public static final double maxAngleDeltaPerFrameDegrees = 30;

    public static final double driveFeedForward = 0.04;

    public static final double voltageFeedForward = 0.1;

    public static final double voltageForMaxSpeed = 12;

    public static final double wheelDiameterMeters = ConversionUtils.inchesToMeters(3.8338048662);
    public static final double driveGearRatio = 1296/209;



    public static final double wheelDistancePerRotation = wheelDiameterMeters * Math.PI;
    public static final double driveConversionPositionFactor = wheelDistancePerRotation / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0; /* Figure why this sixty is here. It was last season */

    public static final double distanceBetweenCentersOfRightAndLeftWheels = ConversionUtils.inchesToMeters(24);
        public static final double distanceBetweenCentersOfFrontAndBackWheels = ConversionUtils.inchesToMeters(24);
        public static final double radiusFromCenterToFarthestSwerveModule = Math
        .sqrt(((distanceBetweenCentersOfRightAndLeftWheels * distanceBetweenCentersOfRightAndLeftWheels)
            + (distanceBetweenCentersOfFrontAndBackWheels * distanceBetweenCentersOfFrontAndBackWheels)));

    public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
        new Translation2d(distanceBetweenCentersOfFrontAndBackWheels / 2,
            distanceBetweenCentersOfRightAndLeftWheels / 2),
        new Translation2d(distanceBetweenCentersOfFrontAndBackWheels / 2,
            -distanceBetweenCentersOfRightAndLeftWheels / 2),
        new Translation2d(-distanceBetweenCentersOfFrontAndBackWheels / 2,
            distanceBetweenCentersOfRightAndLeftWheels / 2),
        new Translation2d(-distanceBetweenCentersOfFrontAndBackWheels / 2,
            -distanceBetweenCentersOfRightAndLeftWheels / 2));

    public static final double maxSpeedMetersPerSecond = 4.3;

    public static final double maxRotationAnglePerSecond = 11.4;

    public static final double translationXSlowModeMultipler = .2;

    public static final double translationYSlowModeMultipler = .2;

    public static final double rotationSlowModeMultipler = .2;
}
