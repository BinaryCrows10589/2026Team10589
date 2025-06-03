package binarycrows.robot.SeasonCode;

import binarycrows.robot.StateTable;
import binarycrows.robot.Utils.ConversionUtils;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
    public static final class SwerveDriveConstants {
        // Driver Interaction constants
        public static final double kMaxSpeedMetersPerSecond = StateTable.getValueAsBoolean("isSim") ? 4.3 : 4.3;
        public static final double kMaxRotationAnglePerSecond = StateTable.getValueAsBoolean("isSim") ? 11.4 : 11.4;

        public static final double kTranslationMaxRateOfChangePerSecond = StateTable.getValueAsBoolean("isSim") ? 8 : 8;
        public static final double kRotationMaxRateOfChangePerSecond = StateTable.getValueAsBoolean("isSim") ? 100 : 100;
        public static final double kDriveFeedForward = 0.04;

        public static final double kMaxAngleDeltaPerFrameDegrees = 30;//30;

        // Name of the CAN Bus the Swerve Drive is on.
        public static final String kCANLoopName = "SwerveDrive"; // To swich to CANivor set the CANLoopName to the CANivors serial number or name

        // Swerve Module Configuration Constants
        public static final String kFrontLeftModuleName = "FrontLeftModule";
        public static final String kFrontRightModuleName = "FrontRightModule";
        public static final String kBackLeftModuleName = "BackLeftModule";
        public static final String kBackRightModuleName = "BackRightModule";

        // Drive Motor CANIDs
        public static final int kFrontLeftDriveMotorCANID = 5;
        public static final int kFrontRightDriveMotorCANID = 6;
        public static final int kBackLeftDriveMotorCANID = 7;
        public static final int kBackRightDriveMotorCANID = 8;

        public static final boolean kFrontLeftDriveMotorInverted =  StateTable.getValueAsBoolean("isSim") ? false : false;
        public static final boolean kFrontRightDriveMotorInverted = StateTable.getValueAsBoolean("isSim") ? false : false;
        public static final boolean kBackLeftDriveMotorInverted = StateTable.getValueAsBoolean("isSim") ? false : false;
        public static final boolean kBackRightDriveMotorInverted = StateTable.getValueAsBoolean("isSim") ? false : false;

        // Turn Motor CANIDs
        public static final int kFrontLeftTurnMotorCANID = 9;
        public static final int kFrontRightTurnMotorCANID = 10;
        public static final int kBackLeftTurnMotorCANID = 11;
        public static final int kBackRightTurnMotorCANID = 12;

        public static final boolean kFrontLeftTurnMotorInverted = StateTable.getValueAsBoolean("isSim") ? false : true;
        public static final boolean kFrontRightTurnMotorInverted = StateTable.getValueAsBoolean("isSim") ? false : true;
        public static final boolean kBackLeftTurnMotorInverted = StateTable.getValueAsBoolean("isSim") ? false : true;
        public static final boolean kBackRightTurnMotorInverted = StateTable.getValueAsBoolean("isSim") ? false : true;

        // Turning Absolute Encoder CANIDs
        public static final int kFrontLeftTurningAbsoluteEncoderCANID = 13;
        public static final int kFrontRightTurningAbsoluteEncoderCANID = 14;
        public static final int kBackLeftTurningAbsoluteEncoderCANID = 15;
        public static final int kBackRightTurningAbsoluteEncoderCANID = 16;

        // Turning Absolute Encoder Offsets in rotations
        public static final double kFrontLeftTurningAbsoluteEncoderOffsetRotations = 0.457275; //Done
        public static final double kFrontRightTurningAbsoluteEncoderOffsetRotations= 0.964600; //
        public static final double kBackLeftTurningAbsoluteEncoderOffsetRotations = 0.925049;
        public static final double kBackRightTurningAbsoluteEncoderOffsetRotations = 0.749756;
    
        // End of Swerve Module Configuration Constants

        public static final int kGyroCANID = 17; 

        // Kinematic Configuration

        public static final double kDistanceBetweenCentersOfRightAndLeftWheels = ConversionUtils.inchesToMeters(23.75);
        public static final double kDistanceBetweenCentersOfFrontAndBackWheels = ConversionUtils.inchesToMeters(23.75);
        public static final double kRadiusFromCenterToFarthestSwerveModule = Math
        .sqrt(((kDistanceBetweenCentersOfRightAndLeftWheels * kDistanceBetweenCentersOfRightAndLeftWheels)
            + (kDistanceBetweenCentersOfFrontAndBackWheels * kDistanceBetweenCentersOfFrontAndBackWheels)));

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kDistanceBetweenCentersOfFrontAndBackWheels / 2,
            kDistanceBetweenCentersOfRightAndLeftWheels / 2),
        new Translation2d(kDistanceBetweenCentersOfFrontAndBackWheels / 2,
            -kDistanceBetweenCentersOfRightAndLeftWheels / 2),
        new Translation2d(-kDistanceBetweenCentersOfFrontAndBackWheels / 2,
            kDistanceBetweenCentersOfRightAndLeftWheels / 2),
        new Translation2d(-kDistanceBetweenCentersOfFrontAndBackWheels / 2,
            -kDistanceBetweenCentersOfRightAndLeftWheels / 2));
        
        public static final String kSwerveDriveModuleStatesLoggerBase = "SwerveDrive/ModuleStates/";
        public static final String kSwerveDriveChassisSpeedLoggerBase = "SwerveDrive/ChassisSpeeds/";
        public static final String kSwerveDriveDesiredChassisSpeedLoggerBase = "SwerveDrive/DesiredChassisSpeeds/";
        public static final double kVoltageForMaxSpeed = 12;
        public static final double kVoltageFeedforward = 0.1;

        /*public static final double[] kElevatorThresholds = {
            ElevatorSubsystem.resolveElevatorPosition(ElevatorPosition.L2),
            ElevatorSubsystem.resolveElevatorPosition(ElevatorPosition.L3),
            ElevatorSubsystem.resolveElevatorPosition(ElevatorPosition.L4)
        };
        public static final double[] kElevatorThresholdVelocityCaps = {
            2.5,
            1.3,
            1
        };
        public static final double[] kElevatorThresholdRotationCaps = {
            15,
            12,
            8,
        };
        */

        public static final int kframesPerCheck = 5;
    }
    public static final class SwerveModuleConstants {
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 4 * 2 * Math.PI;
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 4 * 2 * Math.PI;

        // Motor Control Configuration Values
        public static final double kPModuleDrivePIDValue = 0.0001;
        public static final double kIModuleDrivePIDValue = 0.00000125;
        public static final double kDModuleDrivePIDValue = 0;
        public static final double kFFModuleDrivePIDValue = 0;
        public static final double kIZoneModuleDrivePIDValue = 0.0;

        public static final double kPModuleSIMDrivePIDValue = .1;
        public static final double kIModuleSIMDrivePIDValue = 2;
        public static final double kDModuleSIMDrivePIDValue = 0;

        // If useing TalonFX Swerve these can not be set. Instead please rely on the the max and min voltage.
        public static final double kDriveMotorMinPercentOutput = -1;
        public static final double kDriveMotorMaxPercentOutput = 1;

        public static final double kPModuleTurnPIDValue = .24472;
        public static final double kIModuleTurnPIDValue = 0;
        public static final double kDModuleTurnPIDValue = 0;
        public static final double kFFModuleTurnPIDValue = 0;
        public static final double kIZoneModuleTurnPIDValue = 0.5 / 360; // 1/2 degrees converted to rotations

        public static final double kPModuleSIMTurnPIDValue = 1.5;
        public static final double kIModuleSIMTurnPIDValue = 0;
        public static final double kDModuleSIMTurnPIDValue = 0;

        // If useing TalonFX Swerve these can not be set. Instead please rely on the the max and min voltage.
        public static final double kTurnMotorMinPercentOutput = -.2;
        public static final double kTurnMotorMaxPercentOutput = .2;

        public static final int kDriveMotorMaxAmpsSparkMax = 40;
        public static final int kTurnMotorMaxAmpsSparkMax = 5;
        
        public static final int kDriveMotorMaxVoltageSparkMaxTalonFX = 13;

        public static final int kTurnMotorMaxVoltageSparkMaxTalonFX = 2; 

        // Swerve Module Configuration Values
        public static final double kWheelDiameterMeters = ConversionUtils.inchesToMeters(3.8338048662);//3.9547073083);//Units.inchesToMeters(3.86060506);//Units.inchesToMeters(3.733428923);// 3.83931974016;
        public static final double kDriveGearRatio = 7.125;//7.125;// 6.75/1.0;
        public static final double kTurningGearRatio = 12.8/1;

        public static final double kWheelDistancePerRotation = kWheelDiameterMeters * Math.PI;
        public static final double kDriveConversionPositionFactor = kWheelDistancePerRotation / kDriveGearRatio;
        public static final double kDriveConversionVelocityFactor = kDriveConversionPositionFactor / 60.0; /* Figure why this sixty is here. It was last seasion
            on it work so it does not need to change but why is it here. */
        public static final String kSwerveModuleOutputLoggerBase = "SwerveDrive/Modules/";
    }
}
