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

        public double leftLongitudinalMotorVelocityRPS = 0;
        public double leftLongitudinalMotorAppliedVoltage = 0;
        public double leftLongitudinalMotorSupplyAmps = 0;
        public double leftLongitudinalMotorTorqueAmps = 0;

        public double rightLongitudinalMotorVelocityRPS = 0;
        public double rightLongitudinalMotorAppliedVoltage = 0;
        public double rightLongitudinalMotorSupplyAmps = 0;
        public double rightLongitudinalMotorTorqueAmps = 0;

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
