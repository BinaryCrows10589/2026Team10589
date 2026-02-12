package binarycrows.robot.SeasonCode.SubStateManagers.Turret;

import edu.wpi.first.math.geometry.Rotation2d;

public interface TurretIO {

    public class TurretOutputs {
        public double motorVelocityRPS;
        public double motorAppliedVoltage;
        public double motorSupplyAmps;
        public double motorTorqueAmps;

        public Rotation2d motorRotation;
        public Rotation2d encoderRotation;
        public Rotation2d turretRotation;
        public double turretRotationalVelocityRadPerSec;
        public Rotation2d targetPosition;
        public Rotation2d distanceFromSetpoint;
    }

    public default void update() {}

    public default void setRotorVoltage(double voltage) {}

    public default void setTargetPosition(Rotation2d position) {} // Angle of the turret itself based on absolute encoder, NOT the rotation value of the rotor

    public default void resetMotorToAbsolute() {};

    public default TurretOutputs getOutputs() {return null;};
}
