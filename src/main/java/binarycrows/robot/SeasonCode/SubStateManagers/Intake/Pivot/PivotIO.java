package binarycrows.robot.SeasonCode.SubStateManagers.Intake.Pivot;

import edu.wpi.first.math.geometry.Rotation2d;

public interface PivotIO {
    
    public class PivotOutputs {
        public double leftMotorVelocityRPS = 0;
        public double leftMotorAppliedVoltage = 0;
        public double leftMotorSupplyAmps = 0;
        public double leftMotorTorqueAmps = 0;

        public double rightMotorVelocityRPS = 0;
        public double rightMotorAppliedVoltage = 0;
        public double rightMotorSupplyAmps = 0;
        public double rightMotorTorqueAmps = 0;

        public Rotation2d motorRotation;
        public Rotation2d encoderRotation;
        public Rotation2d pivotRotation;
        public double pivotRotationalVelocityRadPerSec;
        public Rotation2d targetPosition;
        public Rotation2d distanceFromSetpoint;
    }

    public default void update() {}

    public default void setRotorVoltage(double rotorVoltage) {}

    public default void setTargetPosition(Rotation2d position) {}
    public default void resetMotorToAbsolute() {}

}
