package binarycrows.robot.SeasonCode.Constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;

public final class IntakeConstants {
    public final class Rollers {
        public static final InvertedValue masterMotorInverted = InvertedValue.Clockwise_Positive;

        public static final NeutralModeValue motorNeutralMode = NeutralModeValue.Brake;

        public static final double maxMotorVoltage = 12;

        public static final MotorAlignmentValue isSlaveReversed = MotorAlignmentValue.Opposed;

        public static final double intakingMotorVoltage = 2.5;

        public static final double intakeWheelMinVoltage = 2.5;
        public static final double intakeWheelMaxVoltage = 6;

        public static final double overdriveVoltage = 3.5;
    }

    public final class Pivot {
        public static final InvertedValue masterMotorInverted = InvertedValue.Clockwise_Positive;

        public static final NeutralModeValue motorNeutralMode = NeutralModeValue.Coast;

        public static final double maxMotorVoltageUp = 8;
        public static final double maxMotorVoltageDown = -8;

        public static final MotorAlignmentValue isSlaveReversed = MotorAlignmentValue.Opposed;

        public static final double gearRatio = 40;

        public static final Rotation2d pivotEncoderOffset = Rotation2d.kZero;

        public static final double pivotPIDValueP = 0;
        public static final double pivotPIDValueI = 0;
        public static final double pivotPIDValueD = 0;
        public static final double pivotPIDValueFF = 0;


        public static final Rotation2d pivotDownPosition = Rotation2d.fromRotations(0.475342);
        public static final Rotation2d pivotRaisedPosition = Rotation2d.fromRotations(0.352295);
        public static final Rotation2d pivotUpPosition = Rotation2d.fromRotations(0.144775);
        public static final Rotation2d intakeRollerActivateThreshold = Rotation2d.fromRotations(0.416992);

        public static final double manualVoltage = 1.5;
        public static final double manualVoltageFF = 1;

        public static final double torqueCurrentLimit = 60;

        
}
}
