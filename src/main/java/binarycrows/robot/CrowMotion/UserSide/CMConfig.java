package binarycrows.robot.CrowMotion.UserSide;

import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * CMConfig is a centralized configuration class for CrowMotion.
 * <p>
 * This class stores robot, field, and trajectory parameters, and provides static accessors
 * and mutators for consumers and suppliers of these values. 
 * </p>
 */
public class CMConfig {
    private static Supplier<double[]> getRobotPositionMetersAndDegrees;
    private static Supplier<double[]> getRobotVelocityMPSAndDPS;
    private static Consumer<double[]> setRobotVelocityMPSAndDPS;

    private static double drivebaseCircumference;
    private static boolean isBlueAlliance;
    private static Supplier<Boolean> defaultShouldMirror;
    private static double fieldWidthMeters;
    private static double fieldLengthMeters;

    // ==== Motion configuration variables ====
    private static double defaultPointsPerMeter;
    private static double defaultLookAheadMultiplier;
    private static double defaultMaxDesiredTranslationalVelocity;
    private static double defaultMaxDesiredTranslationalAcceleration;
    private static double defaultMaxDesiredTranslationalDeceleration;
    private static double defaultMinStartVelocity;
    private static double defaultEndVelocity;
    private static double defaultDistanceAtEndVelocity;
    private static double maxPossibleAverageSwerveModuleMPS;

    private static double defaultMaxDesiredRotationalVelocity;
    private static double defaultMaxDesiredRotationalAcceleration;
    private static double defaultMaxDesiredRotationalDeceleration;
    private static double defaultAngleCorrectionRange;
    private static double defaultMaxRotationCorrectionVelocityDegrees;
    private static double defaultMinRotationVelocityToMove;
    private static double defaultMaxToleranceDegrees;
    private static double defaultDecelerationBufferDegrees;


    /**
     * <p>
     * Initializes Crow Motion and sets default values for trajectories.
     * </p>
     *
     * @param robotPositionSupplier Supplier providing robot position in meters and degrees [x, y, angle]
     * @param robotVelocitySupplier Supplier providing robot velocity in meters/sec and degrees/sec [vx, vy, omega]
     * @param robotVelocityConsumer Consumer to set robot velocity in meters/sec and degrees/sec [vx, vy, omega]
     * @param distanceBetweenCentersOfRightAndLeftWheels Distance between centers of right and left wheels
     * @param distanceBetweenCentersOfFrontAndBackWheels Distance between centers of front and back wheels
     * @param isBlueAlliance Boolean indicating if the robot is on the blue alliance
     * @param shouldMirrorSupplier Supplier to determine if paths should be mirrored
     * @param fieldWidth Field width in meters
     * @param fieldLength Field length in meters
     *
     * @param pointsPerMeter Default points per meter for trajectory resolution
     * @param lookAheadMultiplier Look-ahead distance multiplier for path following
     * @param maxTransVel Max desired translational velocity (m/s)
     * @param maxTransAccel Max desired translational acceleration (m/s^2)
     * @param maxTransDecel Max desired translational deceleration (m/s^2)
     * @param minStartVel Minimum starting velocity (m/s)
     * @param defaultEndVelocity Default min velocity that will be traveled at for distence at min velocity
     * @param distanceAtEndVel Distance traveled at minimum velocity (m)
     * @param maxAvgSwerveMPS Maximum possible average swerve module speed (m/s)
     * @param maxRotVel Max desired rotational velocity (deg/s)
     * @param maxRotAccel Max desired rotational acceleration (deg/s^2)
     * @param maxRotDecel Max desired rotational deceleration (deg/s^2)
     * @param angleCorrectionRange Range for correcting angle errors (deg)
     * @param maxRotCorrectionVel Max velocity for rotation correction (deg/s)
     * @param minRotVelToMove Minimum rotational velocity to cause motion (deg/s)
     * @param maxToleranceDegrees Maximum allowed angular tolerance (deg)
     * @param decelBufferDegrees Deceleration buffer in degrees
     */
    public static void init(
            Supplier<double[]> robotPositionSupplier,
            Supplier<double[]> robotVelocitySupplier,
            Consumer<double[]> robotVelocityConsumer,
            double distanceBetweenCentersOfRightAndLeftWheels,
            double distanceBetweenCentersOfFrontAndBackWheels,
            boolean isBlueAlliance,
            Supplier<Boolean> shouldMirrorSupplier,
            double fieldWidth,
            double fieldLength,
            double pointsPerMeter,
            double lookAheadMultiplier,
            double maxTransVel,
            double maxTransAccel,
            double maxTransDecel,
            double minStartVel,
            double defaultEndVelocity,
            double distanceAtEndVel,
            double maxAvgSwerveMPS,
            double maxRotVel,
            double maxRotAccel,
            double maxRotDecel,
            double angleCorrectionRange,
            double maxRotCorrectionVel,
            double minRotVelToMove,
            double maxToleranceDegrees,
            double decelBufferDegrees
    ) {
        CMConfig.getRobotPositionMetersAndDegrees = robotPositionSupplier;
        CMConfig.getRobotVelocityMPSAndDPS = robotVelocitySupplier;
        CMConfig.setRobotVelocityMPSAndDPS = robotVelocityConsumer;

        CMConfig.drivebaseCircumference = calculateDrivebaseCircumference(
                distanceBetweenCentersOfRightAndLeftWheels, distanceBetweenCentersOfFrontAndBackWheels);

        CMConfig.isBlueAlliance = isBlueAlliance;
        CMConfig.defaultShouldMirror = shouldMirrorSupplier;
        CMConfig.fieldWidthMeters = fieldWidth;
        CMConfig.fieldLengthMeters = fieldLength;

        CMConfig.defaultPointsPerMeter = pointsPerMeter;
        CMConfig.defaultLookAheadMultiplier = lookAheadMultiplier;
        CMConfig.defaultMaxDesiredTranslationalVelocity = maxTransVel;
        CMConfig.defaultMaxDesiredTranslationalAcceleration = maxTransAccel;
        CMConfig.defaultMaxDesiredTranslationalDeceleration = maxTransDecel;
        CMConfig.defaultMinStartVelocity = minStartVel;
        CMConfig.defaultEndVelocity = defaultEndVelocity;
        CMConfig.defaultDistanceAtEndVelocity = distanceAtEndVel;
        CMConfig.maxPossibleAverageSwerveModuleMPS = maxAvgSwerveMPS;

        CMConfig.defaultMaxDesiredRotationalVelocity = maxRotVel;
        CMConfig.defaultMaxDesiredRotationalAcceleration = maxRotAccel;
        CMConfig.defaultMaxDesiredRotationalDeceleration = maxRotDecel;
        CMConfig.defaultAngleCorrectionRange = angleCorrectionRange;
        CMConfig.defaultMaxRotationCorrectionVelocityDegrees = maxRotCorrectionVel;
        CMConfig.defaultMinRotationVelocityToMove = minRotVelToMove;
        CMConfig.defaultMaxToleranceDegrees = maxToleranceDegrees;
        CMConfig.defaultDecelerationBufferDegrees = decelBufferDegrees;
    }

    private static double calculateDrivebaseCircumference(double distanceBetweenCentersOfRightAndLeftWheels, double distanceBetweenCentersOfFrontAndBackWheels) {
        double majorAxis = Math.max(distanceBetweenCentersOfRightAndLeftWheels, distanceBetweenCentersOfFrontAndBackWheels);
        double minorAxis = Math.min(distanceBetweenCentersOfRightAndLeftWheels, distanceBetweenCentersOfFrontAndBackWheels);
        double h = Math.pow(((majorAxis - minorAxis) / (majorAxis + minorAxis)), 2);

        return (Math.PI * (majorAxis + minorAxis)) * (1 + ((3 * h) / (10 + Math.sqrt(4 - (3 * h)))));
    }

    // === Getters for suppliers/consumers ===
    public static double[] getRobotPositionMetersAndDegrees() {
        return getRobotPositionMetersAndDegrees.get();
    }

    public static double[] getRobotVelocityMPSAndDPS() {
        return getRobotVelocityMPSAndDPS.get();
    }

    public static void setRobotVelocityMPSAndDPS(double x, double y, double rot) {
        setRobotVelocityMPSAndDPS.accept(new double[]{x, y, rot});
    }

    public static double getDrivebaseCircumference() {
        return drivebaseCircumference;
    }

    public static boolean isBlueAlliance() {
        return isBlueAlliance;
    }

    public static boolean shouldMirror() {
        return defaultShouldMirror.get();
    }

    public static double getFieldWidthMeters() {
        return fieldWidthMeters;
    }

    public static double getFieldLengthMeters() {
        return fieldLengthMeters;
    }

    public static double getDefaultPointsPerMeter() { return defaultPointsPerMeter; }
    public static double getDefaultLookAHeadMultiplier() { return defaultLookAheadMultiplier; }
    public static double getDefaultMaxDesiredTranslationalVelocity() { return defaultMaxDesiredTranslationalVelocity; }
    public static double getDefaultMaxDesiredTranslationalAcceleration() { return defaultMaxDesiredTranslationalAcceleration; }
    public static double getDefaultMaxDesiredTranslationalDecceleration() { return defaultMaxDesiredTranslationalDeceleration; }
    public static double getDefaultMinStartVelocity() { return defaultMinStartVelocity; }
    public static double getDefaultEndVelocity() {return defaultEndVelocity;}
    public static double getDefaultDistenceAtEndVelocity() { return defaultDistanceAtEndVelocity; }
    public static double getMaxPossibleAverageSwerveModuleMPS() { return maxPossibleAverageSwerveModuleMPS; }
    public static double getDefaultMaxDesiredRotationalVelocity() { return defaultMaxDesiredRotationalVelocity; }
    public static double getDefaultMaxDesiredRotationalAcceleration() { return defaultMaxDesiredRotationalAcceleration; }
    public static double getDefaultMaxDesiredRotationalDeceleration() { return defaultMaxDesiredRotationalDeceleration; }
    public static double getDefaultAngleCorrectionRange() { return defaultAngleCorrectionRange; }
    public static double getDefaultMaxRotationCorrectionVelocityDegrees() { return defaultMaxRotationCorrectionVelocityDegrees; }
    public static double getDefaultMinRotationVelocityToMove() { return defaultMinRotationVelocityToMove; }
    public static double getDefaultMaxToleranceDegrees() { return defaultMaxToleranceDegrees; }
    public static double getDefaultDecelerationBufferDegrees() { return defaultDecelerationBufferDegrees; }

    
}
