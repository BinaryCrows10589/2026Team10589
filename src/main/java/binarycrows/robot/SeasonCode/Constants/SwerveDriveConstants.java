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

    public static final boolean frontLeftDriveInverted = MetaConstants.isReal;
    public static final boolean frontRightDriveInverted = MetaConstants.isReal;
    public static final boolean backLeftDriveInverted = MetaConstants.isReal;
    public static final boolean backRightDriveInverted = !MetaConstants.isReal;

    public static final boolean frontLeftTurnInverted = MetaConstants.isReal;
    public static final boolean frontRightTurnInverted = MetaConstants.isReal;
    public static final boolean backLeftTurnInverted = MetaConstants.isReal;
    public static final boolean backRightTurnInverted = MetaConstants.isReal;

    public static final double frontLeftTurnEncoderOffset = MetaConstants.isReal ? 0.302490 : 0;
    public static final double frontRightTurnEncoderOffset = MetaConstants.isReal ? 0.369629 : 0;
    public static final double backLeftTurnEncoderOffset = MetaConstants.isReal ? 0.192383 : 0;
    public static final double backRightTurnEncoderOffset = MetaConstants.isReal ? 0.156006 : 0;

    public static final int maxDriveMotorVoltage = 13;
    public static final int maxTurnMotorVoltage = 1;

    public static final double maxSpeedMPS = 4.672;

    // Poorly calculated acceleration: 2.72419825073 MPS

    public static final double turnPIDValueP = MetaConstants.isReal ? .165 : 1;
    public static final double turnPIDValueI = 0;
    public static final double turnPIDValueD = 0;
    public static final double turnPIDValueFF = 0;
    public static final double turnPIDValueIZone = 0.5 / 360; // 1/2 degrees converted to rotations

    public static final double turnGearRatio = 12.1;

    public static final double maxAngleDeltaPerFrameDegrees = 30;

    public static final double driveFeedForward = 0.04;

    public static final double voltageFeedForward = 0.2;
    public static final double mpsLerpTableCutoff = 0.25;


    public static final double voltageForMaxSpeed = 12.1;

    public static final double wheelDiameterMeters = ConversionUtils.inchesToMeters(3.47123485669);
    public static final double driveGearRatio = 1458/209;



    public static final double wheelDistancePerRotation = wheelDiameterMeters * Math.PI;
    public static final double driveConversionPositionFactor = wheelDistancePerRotation / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0; /* Figure why this sixty is here. It was last season */ //Probably to undo the *60 in the swerve module IO when getting MPS

    public static final double distanceBetweenCentersOfRightAndLeftWheels = ConversionUtils.inchesToMeters(24);
    public static final double distanceBetweenCentersOfFrontAndBackWheels = ConversionUtils.inchesToMeters(24);
    public static final double radiusFromCenterToFarthestSwerveModule = Math.sqrt(
        ((distanceBetweenCentersOfRightAndLeftWheels * distanceBetweenCentersOfRightAndLeftWheels) + (
            distanceBetweenCentersOfFrontAndBackWheels * distanceBetweenCentersOfFrontAndBackWheels)));

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
