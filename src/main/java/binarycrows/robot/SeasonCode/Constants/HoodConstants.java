package binarycrows.robot.SeasonCode.Constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;

public class HoodConstants {
    public static final double motorToHoodGearRatio = 20;

    public static final double hoodPIDValueP = 0;
    public static final double hoodPIDValueI = 0;
    public static final double hoodPIDValueD = 0;
    public static final double hoodPIDValueFF = 0;

    public static final double hoodGravityFF = 0; //TODO: With real encoder make sure this is being used correctly. It should be such that the more horizontal the turret is, the more of this gets used.


    public static final double maximumVoltage = 4;

    public static final InvertedValue motorInverted = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue motorNeutralMode = NeutralModeValue.Brake;

    public static final Rotation2d hoodEncoderOffset = Rotation2d.kZero;
}
