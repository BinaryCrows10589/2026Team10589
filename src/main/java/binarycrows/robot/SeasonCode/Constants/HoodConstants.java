package binarycrows.robot.SeasonCode.Constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import binarycrows.robot.Utils.LerpTable;
import edu.wpi.first.math.geometry.Rotation2d;

public class HoodConstants {
    public static final double motorToHoodGearRatio = 1;

    public static final double hoodPIDValueP = 1;
    public static final double hoodPIDValueI = 0;
    public static final double hoodPIDValueD = 0;
    public static final double hoodPIDValueFF = 0;

    public static final double hoodGravityFF = MetaConstants.isReal ? 1.0 : 0; //TODO: With real encoder make sure this is being used correctly. It should be such that the more horizontal the turret is, the more of this gets used.

    public static final boolean useIntegratedPID = false;

    public static final double maximumVoltage = 4;

    public static final InvertedValue motorInverted = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue motorNeutralMode = NeutralModeValue.Brake;

    public static final Rotation2d hoodEncoderOffset = Rotation2d.kZero;

    public static double correctionVelocityRadPerSec = MetaConstants.isReal ? 0 : 0.75;
    public static Rotation2d correctionZone = MetaConstants.isReal ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(20);

    public static double decelerationBufferRad = MetaConstants.isReal ? 0 : 0.025;

    public static double startingVelocityRadPerSec = MetaConstants.isReal ? 0 : 0.1;

    public static double correctionFactorTuningDeltaThresholdRad = MetaConstants.isReal ? 0 : 0.05;

    public static double maxHoodVelocityRadPerSec = MetaConstants.isReal ? 0 : 12;
    public static double minHoodVelocityRadPerSec = correctionVelocityRadPerSec;

    public static double maxAccelerationPerFrameRadPerSecPerSec = MetaConstants.isReal ? 0 : 40;
    public static double maxDecelerationPerFrameRadPerSecPerSec = MetaConstants.isReal ? 0 : 200; // Will need to be significantly higher than max

    public static final LerpTable velocityToVoltageLerpTable = MetaConstants.isReal ? 
    // Real
    new LerpTable(new double[] {}, new double[] {}, true) 
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
}
