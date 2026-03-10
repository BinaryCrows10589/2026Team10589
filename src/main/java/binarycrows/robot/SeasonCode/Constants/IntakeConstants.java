package binarycrows.robot.SeasonCode.Constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;

public final class IntakeConstants {
    public final class Rollers {
        public static final InvertedValue masterMotorInverted = InvertedValue.Clockwise_Positive;

        public static final NeutralModeValue motorNeutralMode = NeutralModeValue.Brake;

        public static final double maxMotorVoltage = 2;

        public static final MotorAlignmentValue isSlaveReversed = MotorAlignmentValue.Opposed;

        public static final double intakingMotorVoltage = 1.5;
    }

    public final class Pivot {
        public static final InvertedValue masterMotorInverted = InvertedValue.Clockwise_Positive;

        public static final NeutralModeValue motorNeutralMode = NeutralModeValue.Coast;

        public static final double maxMotorVoltageUp = 0.5;
        public static final double maxMotorVoltageDown = -0.5;

        public static final MotorAlignmentValue isSlaveReversed = MotorAlignmentValue.Opposed;

        public static final double gearRatio = 40;

        public static final Rotation2d pivotEncoderOffset = Rotation2d.kZero;

        public static final double pivotPIDValueP = 0;
        public static final double pivotPIDValueI = 0;
        public static final double pivotPIDValueD = 0;
        public static final double pivotPIDValueFF = 0;


        public static final Rotation2d pivotDownPosition = Rotation2d.fromRotations(0.844971);
        public static final Rotation2d pivotRaisedPosition = Rotation2d.fromRotations(0.689697);
        public static final Rotation2d pivotUpPosition = Rotation2d.fromRotations(0.463379);

        public static final double manualVoltage = 4;
        public static final double manualVoltageFF = 1;

        public static final double torqueCurrentLimit = 60;
}
}
