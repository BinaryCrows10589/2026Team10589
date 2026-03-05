package binarycrows.robot.SeasonCode.SubStateManagers.Transit;

public interface TransitIO {
    
    public class TransitOutputs {
        public double leftLatitudinalMotorVelocityRPS = 0;
        public double leftLatitudinalMotorAppliedVoltage = 0;
        public double leftLatitudinalMotorSupplyAmps = 0;
        public double leftLatitudinalMotorTorqueAmps = 0;

        public double rightLatitudinalMotorVelocityRPS = 0;
        public double rightLatitudinalMotorAppliedVoltage = 0;
        public double rightLatitudinalMotorSupplyAmps = 0;
        public double rightLatitudinalMotorTorqueAmps = 0;

        public double longitudinalMotorVelocityRPS = 0;
        public double longitudinalMotorAppliedVoltage = 0;
        public double longitudinalMotorSupplyAmps = 0;
        public double longitudinalMotorTorqueAmps = 0;

        public double inAndUpMotorVelocityRPS = 0;
        public double inAndUpMotorAppliedVoltage = 0;
        public double inAndUpMotorSupplyAmps = 0;
        public double inAndUpMotorTorqueAmps = 0;
    }

    public default void update() {}

    public default void setLatitudinalVoltage(double rotorVoltage) {}
    public default void setLongitudinalVoltage(double rotorVoltage) {}
    public default void setInAndUpVoltage(double rotorVoltage) {}

}
