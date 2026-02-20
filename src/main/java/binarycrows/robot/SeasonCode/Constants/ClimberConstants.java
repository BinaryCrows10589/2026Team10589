package binarycrows.robot.SeasonCode.Constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;

public final class ClimberConstants {
    public static final InvertedValue motorInverted = InvertedValue.Clockwise_Positive;

        public static final NeutralModeValue motorNeutralMode = NeutralModeValue.Brake;

        public static final double maxMotorVoltageUp = 6.0;
        public static final double maxMotorVoltageDown = -4.0;

        public static final double climberPIDValueP = 0;
        public static final double climberPIDValueI = 0;
        public static final double climberPIDValueD = 0;
        public static final double climberPIDValueFF = 0;


        public static final Rotation2d climberDownPosition = Rotation2d.fromDegrees(0);
        public static final Rotation2d climberUpPosition = Rotation2d.fromDegrees(90);

        public static final double manualControlVoltage = 4.0;
    }
