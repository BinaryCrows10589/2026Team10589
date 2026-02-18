package binarycrows.robot.SeasonCode.Constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight.RangingMode;

public final class TransitConstants {
    public static final InvertedValue longitudinalMasterMotorInverted = InvertedValue.Clockwise_Positive;
    public static final InvertedValue latitudinalMasterMotorInverted = InvertedValue.Clockwise_Positive;
    public static final InvertedValue inAndUpMotorInverted = InvertedValue.Clockwise_Positive;

    public static final NeutralModeValue longitudinalMasterMotorNeutralMode = NeutralModeValue.Coast;
    public static final NeutralModeValue latitudinalMasterMotorNeutralMode = NeutralModeValue.Coast;
    public static final NeutralModeValue inAndUpMotorNeutralMode = NeutralModeValue.Coast;

    public static final double maxLongitudinalMotorVoltage = 4;
    public static final double maxLatitudinalMotorVoltage = 4;
    public static final double maxInAndUpMotorVoltage = 4;

    public static final MotorAlignmentValue isLongitudinalSlaveReversed = MotorAlignmentValue.Opposed;
    public static final MotorAlignmentValue isLatitudinalSlaveReversed = MotorAlignmentValue.Opposed;

    public final class Sensors {
        public static final RangingMode binFullRangingMode = RangingMode.Short;
        public static final RangingMode outgoingFuelRangingMode = RangingMode.Short;

        public static final double binFullSampleTime = 24;
        public static final double outgoingFuelSampleTime = 24;

        public static final double binFullTrippingDistance = 5;
        public static final double outgoingFuelTrippingDistance = 5;

        public static final int debounceFrames = 10;
    }
}
