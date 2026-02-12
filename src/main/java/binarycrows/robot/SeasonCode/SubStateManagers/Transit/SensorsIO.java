package binarycrows.robot.SeasonCode.SubStateManagers.Transit;

public interface SensorsIO {
    public class SensorsOutputs {
        public double binEmptyReading;
        public double binFullReading;
        public double outgoingFuelReading;
    }

    public default void update() {}
}
