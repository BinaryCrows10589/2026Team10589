package binarycrows.robot.SeasonCode.Constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight.RangingMode;

public final class TransitConstants {
    public static final InvertedValue longitudinalMotorInverted = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue latitudinalMasterMotorInverted = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue inAndUpMotorInverted = InvertedValue.CounterClockwise_Positive;

    public static final NeutralModeValue longitudinalMotorNeutralMode = NeutralModeValue.Coast;
    public static final NeutralModeValue latitudinalMasterMotorNeutralMode = NeutralModeValue.Coast;
    public static final NeutralModeValue inAndUpMotorNeutralMode = NeutralModeValue.Coast;

    public static final double maxLongitudinalMotorVoltage = 8;
    public static final double maxLatitudinalMotorVoltage = 8;
    public static final double maxInAndUpMotorVoltage = 8;

    public static final MotorAlignmentValue isLongitudinalSlaveReversed = MotorAlignmentValue.Opposed;
    public static final MotorAlignmentValue isLatitudinalSlaveReversed = MotorAlignmentValue.Opposed;
    
    public static final double standardLongitudinalMotorVoltagePercent = .3;
    public static final double standardLatitudinalMotorVoltagePercent = .7;
    public static final double standardInAndUpMotorVoltagePercent = .9;

    public static final double standardLongitudinalMotorVoltage = .4;
    public static final double standardLatitudinalMotorVoltage = .6;
    public static final double standardInAndUpMotorVoltage = .8;

    public static final double indexingLongitudinalVoltage = 0;
    public static final double indexingLatitudinalVoltage = 2;
    public static final double indexingInAndUpVoltage = 4;

    public static final double stalledRPSThreshold = 0.1;
    public static final int stalledFramesToInvert = 25;
    public static final int stalledFramesToAbort = 50;

    public final class Sensors {
        public static final RangingMode binFullRangingMode = RangingMode.Short;
        public static final RangingMode outgoingFuelRangingMode = RangingMode.Short;

        public static final double binFullSampleTime = 24;
        public static final double outgoingFuelSampleTime = 24;

        public static final double binFullTrippingDistance = 5;
        public static final double outgoingFuelTrippingDistance = 530;

        public static final int debounceFrames = 10;

        public static final double outgoingFuelIndexingDistance = 120;


    }
}
