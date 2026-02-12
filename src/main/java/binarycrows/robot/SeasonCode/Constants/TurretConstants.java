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


    public static final double correctionVelocityRadPerSec = 0.15;
    public static final Rotation2d correctionZone = Rotation2d.fromDegrees(5);

    public static final double maxTurretVelocityRadPerSec = .4;
    public static final double minTurretVelocityRadPerSec = correctionVelocityRadPerSec;

    public static final double maxAccelerationPerFrameRadPerSecPerSec = .2;
    public static final double maxDecelerationPerFrameRadPerSecPerSec = .2; // Will need to be significantly higher than max

    public static final LerpTable velocityToVoltageLerpTable = MetaConstants.isReal ? 
    // Real
    new LerpTable(new double[] {}, new double[] {}) 
    : 
    // Sim
    new LerpTable(new double[] {}, new double[] {});

    
}
