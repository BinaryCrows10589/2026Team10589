package binarycrows.robot.SeasonCode.Constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import binarycrows.robot.Utils.LerpTable;
import edu.wpi.first.math.geometry.Rotation2d;

public final class TurretConstants {
    public static final double motorToTurretGearRatio = 20f;

    public static final double turretPIDValueP = 0;
    public static final double turretPIDValueI = 0;
    public static final double turretPIDValueD = 0;
    public static final double turretPIDValueFF = 0;

    public static final double maximumVoltage = 6;

    public static final Rotation2d turretEncoderOffset = Rotation2d.fromRotations(0.0);

    public static final InvertedValue motorInverted = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue motorNeutralMode = NeutralModeValue.Brake;


    public static double correctionVelocityRadPerSec = 0;//0.75;
    public static Rotation2d correctionZone = Rotation2d.fromDegrees(0);//20);

    public static double decelerationBufferRad = 0;//0.025;

    public static double startingVelocityRadPerSec = 0;//0.1;

    public static double correctionFactorTuningDeltaThresholdRad = 0;//0.05;

    public static double maxTurretVelocityRadPerSec = 0;//12;
    public static double minTurretVelocityRadPerSec = 0;//correctionVelocityRadPerSec;

    public static double maxAccelerationPerFrameRadPerSecPerSec = 0;//40;
    public static double maxDecelerationPerFrameRadPerSecPerSec = 0;//200; // Will need to be significantly higher than max

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
