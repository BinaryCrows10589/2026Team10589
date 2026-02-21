package binarycrows.robot.SeasonCode.SubStateManagers.Transit;

import com.playingwithfusion.TimeOfFlight;

import binarycrows.robot.SeasonCode.Constants.CANIDs;
import binarycrows.robot.SeasonCode.Constants.TransitConstants;

public class SensorsPWF implements SensorsIO {

    public SensorsOutputs outputs; 

    private TimeOfFlight binFullSensor;
    private TimeOfFlight outgoingFuelSensor;

    private int binFullNumFramesTripped;
    private int outgoingFuelNumFramesTripped;

    public SensorsPWF(SensorsOutputs outputs) {
        // TODO: IF you need help setting the can ids for the TOF ask. Set them on the motor test board first as it is often glitchy. 
        // You should do that next meeting while building happens
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
        /* TODO: ISSUE: Currently you bounce back to not sensing anything the frame after the debouce has finished reading true
            This is incorrect behavior. You want to stay on the last swich. For example if you have been reading positive for 10 frames
            the value should be positive until you have read negitive for 10 frames. Same for the inverse. 
        */
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
