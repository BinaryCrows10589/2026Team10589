package binarycrows.robot.SeasonCode.SubStateManagers.Transit;

import binarycrows.robot.MainStateManager;
import binarycrows.robot.StateRequest;
import binarycrows.robot.StateTable;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.SeasonCode.Constants.TransitConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Hood.HoodStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.Hood.HoodSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Transit.SensorsIO.SensorsOutputs;
import binarycrows.robot.SeasonCode.SubStateManagers.Transit.TransitIO.TransitOutputs;
import binarycrows.robot.SeasonCode.Utils.Shooting;

public class TransitSubStateManager  extends SubStateManager<TransitStateRequest> {
    
    private TransitIO transitIO;
    private TransitOutputs transitOutputs;

    private SensorsIO sensorsIO;
    private SensorsOutputs sensorOutputs;

    private int manualDirection = 0; // 1=forward, 0=off, -1=reverse

    public TransitSubStateManager() {
        super();

        transitOutputs = new TransitOutputs();

        transitIO = new TransitTalonFXS(transitOutputs);
        sensorsIO = new SensorsPWF(sensorOutputs);

        super.defaultState = new StateRequest<TransitStateRequest>(TransitStateRequest.UNPOWERED, StateRequestPriority.NORMAL);
    }

    @Override
    public void recieveStateRequest(StateRequest<TransitStateRequest> request) {
        super.recieveStateRequest(request);
    }

    @Override
    public void periodic() {
        StateTable.logObject("Transit/Outputs", transitOutputs);
        transitIO.update();
        sensorsIO.update();

        switch (activeStateRequest.getStateRequestType()) {
            case POWERED:
                transitIO.setLatitudinalVoltage(TransitConstants.standardLatitudinalMotorVoltage);
                transitIO.setLongitudinalVoltage(TransitConstants.standardLongitudinalMotorVoltage);
                transitIO.setInAndUpVoltage(TransitConstants.standardInAndUpMotorVoltage);
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
            case MANUAL_OVERRIDE:
                transitIO.setLatitudinalVoltage(TransitConstants.standardLatitudinalMotorVoltage * manualDirection);
                transitIO.setLongitudinalVoltage(TransitConstants.standardLongitudinalMotorVoltage * manualDirection);
                transitIO.setInAndUpVoltage(TransitConstants.standardInAndUpMotorVoltage * manualDirection);
                break;
            case SHOOTER:
                if (Shooting.getShooting()) {
                    transitIO.setLatitudinalVoltage(TransitConstants.standardLatitudinalMotorVoltage);
                    transitIO.setLongitudinalVoltage(TransitConstants.standardLongitudinalMotorVoltage);
                    transitIO.setInAndUpVoltage(TransitConstants.standardInAndUpMotorVoltage);
                } else {
                    transitIO.setLatitudinalVoltage(0);
                    transitIO.setLongitudinalVoltage(0);
                    transitIO.setInAndUpVoltage(0);
                }
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
        manualDirection = 1;
    }
    public void manualReverse() {
        manualDirection = -1;
    }
    public void manualOff() {
        manualDirection = 0;
    }

}
