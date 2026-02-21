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

    private int manualDirection = 0; // 1=forward, 0=off, -1=reverse
    private int shooterDirection = 0;

    public TransitSubStateManager() {
        super();

        outputs = new TransitOutputs();

        transitIO = new TransitTalonFXS(outputs);

        super.defaultState = new StateRequest<TransitStateRequest>(TransitStateRequest.UNPOWERED, StateRequestPriority.NORMAL);
    }

    @Override
    public void recieveStateRequest(StateRequest<TransitStateRequest> request) {
        super.recieveStateRequest(request);
    }

    @Override
    public void periodic() {
        StateTable.logObject("Transit/Outputs", outputs);
        transitIO.update();


        // TODO: Question, I unserstand al lbut the last one. What is shooter in this case?
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
            case MANUAL_OVERRIDE:
                transitIO.setLatitudinalVoltage(TransitConstants.maxLatitudinalMotorVoltage * manualDirection);
                transitIO.setLongitudinalVoltage(TransitConstants.maxLongitudinalMotorVoltage * manualDirection);
                transitIO.setInAndUpVoltage(TransitConstants.maxInAndUpMotorVoltage * manualDirection);
            case SHOOTER:
                transitIO.setLatitudinalVoltage(TransitConstants.maxLatitudinalMotorVoltage * shooterDirection);
                transitIO.setLongitudinalVoltage(TransitConstants.maxLongitudinalMotorVoltage * shooterDirection);
                transitIO.setInAndUpVoltage(TransitConstants.maxInAndUpMotorVoltage * shooterDirection);
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

    public void putShooterDirection(int direction) {
        shooterDirection = direction;
    }
}
