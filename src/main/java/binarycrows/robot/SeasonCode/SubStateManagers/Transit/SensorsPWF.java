package binarycrows.robot.SeasonCode.SubStateManagers.Transit;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import binarycrows.robot.SeasonCode.Constants.CANIDs;
import binarycrows.robot.SeasonCode.Constants.TransitConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Transit.SensorsIO.SensorsOutputs;

public class SensorsPWF implements SensorsIO {

    public SensorsOutputs outputs; 

    private TimeOfFlight binEmptySensor;
    private TimeOfFlight binFullSensor;
    private TimeOfFlight outgoingFuelSensor;

    public SensorsPWF(SensorsOutputs outputs) {
        this.outputs = outputs;
        binEmptySensor = new TimeOfFlight(CANIDs.RIO.binEmptyTOFSensor);
        binFullSensor = new TimeOfFlight(CANIDs.RIO.binFullTOFSensor);
        outgoingFuelSensor = new TimeOfFlight(CANIDs.RIO.outgoingFuelTOFSensor);

        binEmptySensor.setRangingMode(TransitConstants.Sensors.binEmptyRangingMode, TransitConstants.Sensors.binEmptySampleTime);
        binFullSensor.setRangingMode(TransitConstants.Sensors.binFullRangingMode, TransitConstants.Sensors.binFullSampleTime);
        outgoingFuelSensor.setRangingMode(TransitConstants.Sensors.outgoingFuelRangingMode, TransitConstants.Sensors.outgoingFuelSampleTime);
    }

    @Override
    public void update() {
        outputs.binEmptyReading = binEmptySensor.getRangeSigma();
        outputs.binFullReading = binFullSensor.getRangeSigma();
        outputs.outgoingFuelReading = outgoingFuelSensor.getRangeSigma(); 
    }
}
