package binarycrows.robot.SeasonCode.SubStateManagers.Transit;

import com.playingwithfusion.TimeOfFlight;

import binarycrows.robot.SeasonCode.Constants.CANIDs;
import binarycrows.robot.SeasonCode.Constants.TransitConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.CANdle.CANdleStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.CANdle.CANdleSubStateManager;

public class SensorsPWF implements SensorsIO {

    public SensorsOutputs outputs; 

    private TimeOfFlight binFullSensor;
    private TimeOfFlight outgoingFuelSensor;

    private int binFullNumFramesDifferent;
    private int outgoingFuelNumFramesDifferent;

    public SensorsPWF(SensorsOutputs outputs) {
        // TODO: IF you need help setting the can ids for the TOF ask. Set them on the motor test board first as it is often glitchy. 
        // You should do that next meeting while building happens
        // Clanker says: connect to http://<roboRIO-IP>:5812
        // https://www.playingwithfusion.com/include/getfile.php?fileid=7091
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
        boolean binFullTrippedThisFrame = outputs.binFullReading >= TransitConstants.Sensors.binFullTrippingDistance;

        if (outputs.binFullTripped != binFullTrippedThisFrame) binFullNumFramesDifferent++;
        else binFullNumFramesDifferent = 0;

        if (binFullNumFramesDifferent > TransitConstants.Sensors.debounceFrames) {
            binFullNumFramesDifferent = 0;
            outputs.binFullTripped = binFullTrippedThisFrame;
        }

        boolean outgoingFuelTrippedThisFrame = outputs.outgoingFuelReading >= TransitConstants.Sensors.outgoingFuelTrippingDistance;

        if (outputs.outgoingFuelTripped != outgoingFuelTrippedThisFrame) outgoingFuelNumFramesDifferent++;
        else outgoingFuelNumFramesDifferent = 0;

        if (outgoingFuelNumFramesDifferent > TransitConstants.Sensors.debounceFrames) {
            outgoingFuelNumFramesDifferent = 0;
            outputs.outgoingFuelTripped = outgoingFuelTrippedThisFrame;
        }

        if (outputs.binFullTripped) {
            CANdleSubStateManager.setLEDs(CANdleStateRequest.HOPPER_FULL);
        } else if (outputs.outgoingFuelTripped) {
            CANdleSubStateManager.setLEDs(CANdleStateRequest.HOPPER_PARTIAL);
        } else {
            CANdleSubStateManager.setLEDs(CANdleStateRequest.HOPPER_EMPTY);
        }

    }
}
