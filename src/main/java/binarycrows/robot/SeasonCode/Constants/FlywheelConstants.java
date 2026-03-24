package binarycrows.robot.SeasonCode.Constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import binarycrows.robot.Utils.LerpTable;

public final class FlywheelConstants {
    public static final InvertedValue masterMotorInverted = InvertedValue.CounterClockwise_Positive;

    public static final NeutralModeValue motorNeutralMode = NeutralModeValue.Coast;

    public static final double maxMotorVoltage = 11;

    public static final double motorShootingVoltage = 10;

    public static final double reverseVoltage = 0.25;

    public static final double idleRecoveryVoltage = 0;
    public static final double idleMinVelocityRPS = 0;

    public static final MotorAlignmentValue isSlaveReversed = MotorAlignmentValue.Opposed;

    public static final double gearRatio = 2;

    public static final double baseShooterFF = 0;

    public static final double shooterFFIncrement = 0.1;

    public static final LerpTable rpsToVoltage = new LerpTable(
        new double[] {
            0,0.93359375,3.06640625,4.20703125,5.8203125,7.751953125,9.642578125,11.0078125,12.63867188,14.07421875,16.08789063,17.55664063,19.609375,21.04492188,22.95898438,24.19335938,26.11328125,27.9375,29.15429688,31.59765625,32.59960938,34.3203125,35.28515625,37.74414063,39.15820313,41.59765625,42.5625,44.11523438,45.83203125,47.18164063,49.06445313,51.44726563,52.70117188,54.66015625,56.56835938,58.10351563,59.53320313,61.30664063,62.48632813,64.81835938,66.54101563,68.31445313,70.58984375,72.40820313,73.17773438,74.34375,76.8671875,77.640625,79.76171875,81.55664063,83.30273438,83.92382813,86.91992188,87.47851563,86.90429688
        }, 
        new double[] {
            0,0.4,0.6,0.8,1,1.2,1.4,1.6,1.8,2,2.2,2.4,2.6,2.8,3,3.2,3.4,3.6,3.8,4,4.2,4.4,4.6,4.8,5,5.2,5.4,5.6,5.8,6,6.2,6.4,6.6,6.8,7,7.2,7.4,7.6,7.8,8,8.2,8.4,8.6,8.8,9,9.2,9.4,9.6,9.8,10,10.2,10.4,10.6,10.8,10.999999999999993
        }, 
        false);
}
