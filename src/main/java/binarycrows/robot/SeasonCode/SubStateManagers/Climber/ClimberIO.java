package binarycrows.robot.SeasonCode.SubStateManagers.Climber;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ClimberIO {
    
    public class ClimberOutputs {
        public double motorVelocityRPS = 0;
        public double motorAppliedVoltage = 0;
        public double motorSupplyAmps = 0;
        public double motorTorqueAmps = 0;

        public Rotation2d motorRotation;
        public Rotation2d targetPosition;
        public Rotation2d distanceFromSetpoint;
    }

    public default void update() {}

    public default void setRotorVoltage(double rotorVoltage) {}

    public default void setTargetPosition(Rotation2d position) {}

}
