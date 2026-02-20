package binarycrows.robot.SeasonCode.SubStateManagers.Transit;

import binarycrows.robot.MainStateManager;
import binarycrows.robot.StateRequest;
import binarycrows.robot.StateTable;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.SeasonCode.Constants.TransitConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Hood.HoodStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.Hood.HoodSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Transit.TransitIO.TransitOutputs;

public class TransitSubStateManager  extends SubStateManager<TransitStateRequest> {
    
    private TransitIO transitIO;
    private TransitOutputs outputs;

    public TransitSubStateManager() {
        super();

        outputs = new TransitOutputs();

        transitIO = new TransitTalonFXS(outputs);

        super.defaultState = new StateRequest<TransitStateRequest>(TransitStateRequest.UNPOWERED, StateRequestPriority.NORMAL);
    }

    @Override
    public void recieveStateRequest(StateRequest<TransitStateRequest> request) {
        //TODO: Add logic here for manual override and automatic based on shooting
        super.recieveStateRequest(request);
    }

    @Override
    public void periodic() {
        StateTable.logObject("Transit/Outputs", outputs);
        transitIO.update();

        switch (activeStateRequest.getStateRequestType()) {
            case POWERED:
                transitIO.setLatitudinalVoltage(TransitConstants.maxLatitudinalMotorVoltage);
                transitIO.setLongitudinalVoltage(TransitConstants.maxLongitudinalMotorVoltage);
                transitIO.setInAndUpVoltage(TransitConstants.maxInAndUpMotorVoltage);
                break;
            case UNPOWERED:
                transitIO.setLatitudinalVoltage(0);
                transitIO.setLongitudinalVoltage(0);
                transitIO.setInAndUpVoltage(0);
                break;
            case REVERSE:
                transitIO.setLatitudinalVoltage(-TransitConstants.maxLatitudinalMotorVoltage);
                transitIO.setLongitudinalVoltage(-TransitConstants.maxLongitudinalMotorVoltage);
                transitIO.setInAndUpVoltage(-TransitConstants.maxInAndUpMotorVoltage);
                break;
        }
        
    }

    public String toString() {
        return "TransitSubStateManager";
    }

    public static TransitSubStateManager getInstance() {
        return (TransitSubStateManager) MainStateManager.getInstance().resolveSubStateManager(TransitStateRequest.class);
    }

    public void manualForward() {

    }
    public void manualReverse() {
        
    }
    public void manualOff() {
        
    }
}
