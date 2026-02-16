package binarycrows.robot.SeasonCode.SubStateManagers.Transit;

public interface SensorsIO {
    public class SensorsOutputs {
        public double binFullReading;
        public double outgoingFuelReading;

        public boolean binFullValid;
        public boolean outgoingFuelValid;

        public boolean binFullTripped;
        public boolean outgoingFuelTripped;
    }

    public default void update() {}
}
