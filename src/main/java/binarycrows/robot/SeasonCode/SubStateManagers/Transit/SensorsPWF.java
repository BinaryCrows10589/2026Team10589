package binarycrows.robot.SeasonCode.SubStateManagers.Transit;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import binarycrows.robot.SeasonCode.Constants.CANIDs;
import binarycrows.robot.SeasonCode.Constants.TransitConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Transit.SensorsIO.SensorsOutputs;

public class SensorsPWF implements SensorsIO {

    public SensorsOutputs outputs; 

    private TimeOfFlight binFullSensor;
    private TimeOfFlight outgoingFuelSensor;

    private int binFullNumFramesTripped;
    private int outgoingFuelNumFramesTripped;

    public SensorsPWF(SensorsOutputs outputs) {
        this.outputs = outputs;
        binFullSensor = new TimeOfFlight(CANIDs.RIO.binFullTOFSensor);
        outgoingFuelSensor = new TimeOfFlight(CANIDs.RIO.outgoingFuelTOFSensor);

        binFullSensor.setRangingMode(TransitConstants.Sensors.binFullRangingMode, TransitConstants.Sensors.binFullSampleTime);
        outgoingFuelSensor.setRangingMode(TransitConstants.Sensors.outgoingFuelRangingMode, TransitConstants.Sensors.outgoingFuelSampleTime);
    }

    @Override
    public void update() {
        outputs.binFullReading = binFullSensor.getRangeSigma();
        outputs.outgoingFuelReading = outgoingFuelSensor.getRangeSigma(); 

        outputs.binFullValid = binFullSensor.isRangeValid();
        outputs.outgoingFuelValid = outgoingFuelSensor.isRangeValid();

        // Debounce
        if (outputs.binFullReading >= TransitConstants.Sensors.binFullTrippingDistance && !outputs.binFullTripped) {
            binFullNumFramesTripped++;
            if (binFullNumFramesTripped > TransitConstants.Sensors.debounceFrames) {
                outputs.binFullTripped = true;
                binFullNumFramesTripped = 0;
            }
        } else {
            binFullNumFramesTripped = 0;
            if (outputs.binFullReading < TransitConstants.Sensors.binFullTrippingDistance)
                outputs.binFullTripped = false;
        }

        if (outputs.outgoingFuelReading >= TransitConstants.Sensors.outgoingFuelTrippingDistance && !outputs.outgoingFuelTripped) {
            outgoingFuelNumFramesTripped++;
            if (outgoingFuelNumFramesTripped > TransitConstants.Sensors.debounceFrames) {
                outputs.outgoingFuelTripped = true;
                outgoingFuelNumFramesTripped = 0;
            }
        } else {
            outgoingFuelNumFramesTripped = 0;
            if (outputs.outgoingFuelReading < TransitConstants.Sensors.outgoingFuelTrippingDistance)
                outputs.outgoingFuelTripped = false;
        }


    }
}
