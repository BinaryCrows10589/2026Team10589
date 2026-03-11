package binarycrows.robot.SeasonCode.Constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import binarycrows.robot.Utils.LerpTable;
import edu.wpi.first.math.geometry.Rotation2d;

public final class TurretConstants {

    public record TurretControlConstants(
        double correctionVelocityRadPerSec,
        Rotation2d correctionZone,
        double decelerationBufferRad,
        double startingVelocityRadPerSec,
        double correctionFactorTuningDeltaThresholdRad,
        double maxTurretVelocityRadPerSec,
        double minTurretVelocityRadPerSec,
        double maxAccelerationPerFrameRadPerSecPerSec,
        double maxDecelerationPerFrameRadPerSecPerSec
    ) {public TurretControlConstants {}}

    public static final double motorToTurretGearRatio = 20;

    public static final double turretPIDValueP = 0;
    public static final double turretPIDValueI = 0;
    public static final double turretPIDValueD = 0;
    public static final double turretPIDValueFF = 0;

    public static final double maximumVoltage = 4;

    public static final Rotation2d turretEncoderOffset = Rotation2d.fromRotations(0.501709); // Encoder reading when turret is straight forward

    public static final InvertedValue motorInverted = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue motorNeutralMode = NeutralModeValue.Coast;

    public static final TurretControlConstants normalTurretControlConstants = new TurretControlConstants(
        MetaConstants.isReal ? 2 : 0.75, // Correction velocity (rad/sec)
        MetaConstants.isReal ? Rotation2d.fromDegrees(20) : Rotation2d.fromDegrees(20), // Correction zone

        MetaConstants.isReal ? 0.4 : 0.025, // Deceleration buffer rad

        MetaConstants.isReal ? 4 : 0.1, // Starting velocity rad per sec

        MetaConstants.isReal ? 0.05 : 0.05, // Correction factor tuning delta threshold rad

        MetaConstants.isReal ? 8 : 12, // Max turret velocity rad per sec
        MetaConstants.isReal ? 2 : 0.75, // Min turret velocity rad per sec

        MetaConstants.isReal ? 10 : 40, // Max acceleration per frame (rad/s^2)
        MetaConstants.isReal ? 20 : 200 // Max deceleration per frame (rad/s/s) (will need to be significantly higher than max)
    );
    public static final TurretControlConstants overextensionTurretControlConstants = new TurretControlConstants(
        MetaConstants.isReal ? 1.25 : 0.75, // Correction velocity rad per sec
        MetaConstants.isReal ? Rotation2d.fromDegrees(25) : Rotation2d.fromDegrees(20), // Correction zone

        MetaConstants.isReal ? 0.025 : 0.025, // Deceleration buffer rad

        MetaConstants.isReal ? 1.75 : 0.1, // Starting velocity rad per sec

        MetaConstants.isReal ? 0.05 : 0.05, // Correction factor tuning delta threshold rad

        MetaConstants.isReal ? 3 : 12, // Max turret velocity rad per sec
        MetaConstants.isReal ? 1.75 : 0.75, // Min turret velocity rad per sec

        MetaConstants.isReal ? 5 : 40, // Max acceleration per frame rad per sec
        MetaConstants.isReal ? 50 : 200 // Max deceleration per frame rad per sec (will need to be significantly higher than max)
    );

    public static final double forwardOverextensionRad = Math.PI / 4.0 * (0.6);
    public static final double reverseOverextensionRad = Math.PI / 4.0 * (0.8);

    public static final double forwardForceNormalRangeDistanceRotations = .5 + (0.125 * (0.6));
    public static final double reverseForceNormalRangeDistanceRotations = -.5 - (0.125 * (0.8));

    public static final Rotation2d encoderHalfCircleDistance = Rotation2d.fromRotations(0.890869).minus(turretEncoderOffset);

    public static final double manualVoltage = .25;

    public static final double velocityToVoltage(double velocityRadPerSec) {
        return 0.483*velocityRadPerSec + 0.272;
    }

    public static final LerpTable velocityToVoltageLerpTable = MetaConstants.isReal ? 
    // Real
    new LerpTable(new double[] {0, 0}, new double[] {0, 0}, true) 
    : 
    // Sim
    new LerpTable(
    new double[] {
        0,
        2.632,
        5.265,
        7.897,
        10.530,
        13.162,
        15.794,
        18.427,
        21.059,
        23.691,
        26.324,
        28.956,
        31.589
    }, 
    new double[] {
        0,
        1,
        2,
        3,
        4,
        5,
        6,
        7,
        8,
        9,
        10,
        11,
        12
    }, true);

    public static final double torqueCurrentLimit = 100;

    public static final Rotation2d encoderReadingAtMinRotation =  TurretConstants.turretEncoderOffset.minus(TurretConstants.encoderHalfCircleDistance);

    
}
