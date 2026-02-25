package binarycrows.robot.SeasonCode.Constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import binarycrows.robot.Utils.LerpTable;

public final class FlywheelConstants {
    public static final InvertedValue masterMotorInverted = InvertedValue.Clockwise_Positive;

    public static final NeutralModeValue motorNeutralMode = NeutralModeValue.Coast;

    public static final double maxMotorVoltage = .5;

    public static final double reverseVoltage = 0.25;

    public static final double idleRecoveryVoltage = 4.0;
    public static final double idleMinVelocityRPS = 25;

    public static final MotorAlignmentValue isSlaveReversed = MotorAlignmentValue.Opposed;

    public static final double gearRatio = 2;

    public static final double baseShooterFF = 0;

    public static final double shooterFFIncrement = 0.1;

    public static final LerpTable rpmToVoltage = new LerpTable(
        new double[] {
            0
        }, 
        new double[] {
            0
        }, 
        false);
}
