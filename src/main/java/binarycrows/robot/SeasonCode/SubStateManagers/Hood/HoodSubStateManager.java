package binarycrows.robot.SeasonCode.SubStateManagers.Hood;

import binarycrows.robot.MainStateManager;
import binarycrows.robot.StateRequest;
import binarycrows.robot.StateTable;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.SeasonCode.Constants.HoodConstants;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Hood.HoodIO.HoodOutputs;
import binarycrows.robot.SeasonCode.Utils.Shooting;
import edu.wpi.first.math.geometry.Rotation2d;

public class HoodSubStateManager extends SubStateManager<HoodStateRequest> {
    
    private Hood hood;
    private HoodOutputs outputs;


    private Rotation2d manualOverrideTargetRotation = HoodConstants.hoodStartingPosition;

    private Rotation2d targetPosition = HoodConstants.hoodStartingPosition;

    private int manualDirection = 0; // 1=up, 0=none, -1=down

    public HoodSubStateManager() {
        super();

        outputs = new HoodOutputs();

        hood = new Hood(MetaConstants.isReal ? 
        HoodConstants.useIntegratedPID ? new HoodTalonFXSIntegratedPID(outputs) : new HoodTalonFXSWPILibPID(outputs)
        : new HoodSim(outputs));

        super.defaultState = new StateRequest<HoodStateRequest>(HoodStateRequest.RETRACTED, StateRequestPriority.NORMAL);
    }

    @Override
    public void recieveStateRequest(StateRequest<HoodStateRequest> request) {
        super.recieveStateRequest(request);
        if (activeStateRequest.getStateRequestType() == HoodStateRequest.MANUAL_OVERRIDE) {
            manualOverrideTargetRotation = outputs.hoodRotation;
        }
    }

    @Override
    public void periodic() {
        StateTable.logObject("Hood/Outputs", outputs);
        switch (activeStateRequest.getStateRequestType()) {
            case MANUAL_OVERRIDE:
                manualOverrideTargetRotation = manualOverrideTargetRotation.plus(HoodConstants.manualAngleIncrement.times(manualDirection));
                targetPosition = manualOverrideTargetRotation;
                break;
            case SHOOT_ON_THE_MOVE:
                targetPosition = Rotation2d.fromRadians(Shooting.hoodAngleRad);
                break;
            case RETRACTED:
                targetPosition = Rotation2d.kZero;
                break;
        }
        hood.update();
        controlVoltage();
        
    }

    private void controlPID() {
        hood.setPIDTarget(targetPosition);
    }
    private void controlVoltage() {
        double voltage = 0;
        double delta = targetPosition.getDegrees() - outputs.hoodRotation.getDegrees();

        if (delta > 20) voltage = 0.02;
        else if (delta > 5) voltage = 0.002;
        else if (delta < -5) voltage = -0.002;
        else if (delta < -20) voltage = -0.02;
        else controlPID();

        if (voltage != 0) hood.setVoltage(voltage);
    }
    private void controlLikeTurret() {
        hood.setTargetLikeTurret(targetPosition);
    }

    public static HoodSubStateManager getInstance() {
        return (HoodSubStateManager) MainStateManager.getInstance().resolveSubStateManager(HoodStateRequest.class);
    }

    public double getDeltaRad() {
        return outputs.distanceFromSetpoint.getRadians();
    }


    public void manualStop() {
        manualDirection = 0;
    }

    public void manualUp() {
        manualDirection = 1;
    }

    public void manualDown() {
        manualDirection = -1;
    } 
}
