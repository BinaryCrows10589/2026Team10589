package binarycrows.robot.SeasonCode.SubStateManagers.Hood;

import edu.wpi.first.math.geometry.Rotation2d;

public interface HoodIO {

    //TODO: when making hood class, add: 
    // standard pids; 
    // voltage if statements w/ integrated pid catch; 
    // and evil turret control mode

    //TODO: All need gravity feedforward, which is tuned value * sin of angle of hood relative to the horizontal or whatever it is

    public class HoodOutputs {
        public double motorVelocityRPS;
        public double motorAppliedVoltage;
        public double motorSupplyAmps;
        public double motorTorqueAmps;

        public Rotation2d motorRotation;
        public Rotation2d encoderRotation;
        public Rotation2d hoodRotation;
        public double hoodRotationalVelocityRadPerSec;
        public Rotation2d targetPosition;
        public Rotation2d distanceFromSetpoint;
    }

    public default void update() {}

    public default void setRotorVoltage(double voltage) {}

    public default void setTargetPosition(Rotation2d position) {} // Angle of the hood itself based on absolute encoder, NOT the rotation value of the rotor

    public default void resetMotorToAbsolute() {};
}
