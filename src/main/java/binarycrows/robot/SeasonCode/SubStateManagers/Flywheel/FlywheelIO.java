package binarycrows.robot.SeasonCode.SubStateManagers.Flywheel;

public interface FlywheelIO {
    
    public class FlywheelOutputs {
        public double leftMotorVelocityRPS = 0;
        public double leftMotorAppliedVoltage = 0;
        public double leftMotorSupplyAmps = 0;
        public double leftMotorTorqueAmps = 0;

        public double rightMotorVelocityRPS = 0;
        public double rightMotorAppliedVoltage = 0;
        public double rightMotorSupplyAmps = 0;
        public double rightMotorTorqueAmps = 0;
    }

    public default void update() {}

    public default void setRotorVoltage(double rotorVoltage) {}

}
