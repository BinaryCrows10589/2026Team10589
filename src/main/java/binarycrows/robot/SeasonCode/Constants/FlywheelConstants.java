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

    public static final double idleRecoveryVoltage = .5;
    public static final double idleMinVelocityRPS = 25;

    public static final MotorAlignmentValue isSlaveReversed = MotorAlignmentValue.Opposed;

    public static final double gearRatio = 2;

    public static final double baseShooterFF = 0;

    public static final double shooterFFIncrement = 0.1;

    public static final LerpTable rpmToVoltage = new LerpTable(
        new double[] {
            0, 
            0, 
            1.330078125, 
            2.697265625, 
            4.361328125, 
            6.046875, 
            7.673828125, 
            9.310546875, 
            10.658203125, 
            12.25, 
            13.83984375, 
            15.689453125, 
            17.5234375, 
            19.05078125, 
            21.03125, 
            22.357421875, 
            24.328125, 
            25.75390625, 
            27.8203125, 
            29.603515625, 
            30.541015625, 
            32.42578125, 
            34.03125, 
            35.9765625, 
            37.33984375, 
            39.1171875, 
            40.830078125, 
            42.791015625, 
            44.8984375, 
            46.203125, 
            47.529296875, 
            49.25390625, 
            51.00390625, 
            53.025390625, 
            54.576171875, 
            56.5078125, 
            58.046875, 
            59.279296875, 
            61.4453125, 
            63.01171875, 
            64.802734375, 
            66.26953125, 
            67.771484375, 
            70.146484375, 
            72.005859375, 
            73.30078125, 
            74.982421875, 
            76.41796875, 
            78.328125, 
            80.05078125, 
            82.568359375, 
            83.716796875, 
            85.080078125, 
            87.39453125, 
            89.345703125, 
            90.095703125
        }, 
        new double[] {
            0, 
            0.2, 
            0.4, 
            0.6000000000000001, 
            0.8, 
            1, 
            1.2, 
            1.4, 
            1.5999999999999999, 
            1.7999999999999998, 
            1.9999999999999998, 
            2.1999999999999997, 
            2.4, 
            2.6, 
            2.8000000000000003, 
            3.0000000000000004, 
            3.2000000000000006, 
            3.400000000000001, 
            3.600000000000001, 
            3.800000000000001, 
            4.000000000000001, 
            4.200000000000001, 
            4.400000000000001, 
            4.600000000000001, 
            4.800000000000002, 
            5.000000000000002, 
            5.200000000000002, 
            5.400000000000002, 
            5.600000000000002, 
            5.8000000000000025, 
            6.000000000000003, 
            6.200000000000003, 
            6.400000000000003, 
            6.600000000000003, 
            6.800000000000003, 
            7.0000000000000036, 
            7.200000000000004, 
            7.400000000000004, 
            7.600000000000004, 
            7.800000000000004, 
            8.000000000000004, 
            8.200000000000003, 
            8.400000000000002, 
            8.600000000000001, 
            8.8, 
            9, 
            9.2, 
            9.399999999999999, 
            9.599999999999998, 
            9.799999999999997, 
            9.999999999999996, 
            10.199999999999996, 
            10.399999999999995, 
            10.599999999999994, 
            10.799999999999994, 
            10.999999999999993
        }, 
        false);
}
