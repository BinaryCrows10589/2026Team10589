package binarycrows.robot.SeasonCode.Constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;

public final class IntakeConstants {
    public final class Rollers {
        public static final InvertedValue masterMotorInverted = InvertedValue.Clockwise_Positive;

        public static final NeutralModeValue motorNeutralMode = NeutralModeValue.Brake;

        public static final double maxMotorVoltage = .5;

        public static final MotorAlignmentValue isSlaveReversed = MotorAlignmentValue.Opposed;

        public static final double intakingMotorVoltage = 0.25;
    }

    public final class Pivot {
        public static final InvertedValue masterMotorInverted = InvertedValue.Clockwise_Positive;

        public static final NeutralModeValue motorNeutralMode = NeutralModeValue.Brake;

        public static final double maxMotorVoltageUp = 0.5;
        public static final double maxMotorVoltageDown = -0.5;

        public static final MotorAlignmentValue isSlaveReversed = MotorAlignmentValue.Opposed;

        public static final double gearRatio = 30;

        public static final Rotation2d pivotEncoderOffset = Rotation2d.kZero;

        public static final double pivotPIDValueP = 0;
        public static final double pivotPIDValueI = 0;
        public static final double pivotPIDValueD = 0;
        public static final double pivotPIDValueFF = 0;


        public static final Rotation2d pivotDownPosition = Rotation2d.fromDegrees(0);
        public static final Rotation2d pivotRaisedPosition = Rotation2d.fromDegrees(20);
        public static final Rotation2d pivotUpPosition = Rotation2d.fromDegrees(90);

        public static final double manualVoltage = 4;
        public static final double manualVoltageFF = 1;

        public static final double statorCurrentLimit = 60;
}
}
