package binarycrows.robot.SeasonCode.SubStateManagers.Climber;

import binarycrows.robot.MainStateManager;
import binarycrows.robot.StateRequest;
import binarycrows.robot.StateTable;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.Enums.StateRequestStatus;
import binarycrows.robot.SeasonCode.Constants.ClimberConstants;
import binarycrows.robot.SeasonCode.Constants.FlywheelConstants;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Climber.ClimberIO.ClimberOutputs;
import binarycrows.robot.SeasonCode.SubStateManagers.Hood.HoodStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.Hood.HoodSubStateManager;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class ClimberSubStateManager extends SubStateManager<ClimberStateRequest> {
    
    private ClimberIO climberIO;
    private ClimberOutputs outputs;

    private int manualOverrideDirection = 0; // 1=up, 0=none, -1=down

    public ClimberSubStateManager() {
        super();
        outputs = new ClimberOutputs();

        climberIO = new ClimberTalonFX(outputs);

        super.defaultState = new StateRequest<ClimberStateRequest>(ClimberStateRequest.DOWN, StateRequestPriority.NORMAL);
    }

    @Override
    public void recieveStateRequest(StateRequest<ClimberStateRequest> stateRequest) {
        if (stateRequest.getStateRequestType() == ClimberStateRequest.RESTORE_CLOSEST) {
            double distanceToUpPositionRad = Math.abs(ClimberConstants.climberUpPosition.getRadians()-outputs.motorRotation.getRadians());
            double distanceToDownPositionRad = Math.abs(ClimberConstants.climberDownPosition.getRadians()-outputs.motorRotation.getRadians());

            stateRequest = new StateRequest<ClimberStateRequest>(
                (distanceToUpPositionRad < distanceToDownPositionRad) ? ClimberStateRequest.UP : ClimberStateRequest.DOWN, 
            stateRequest.getPriority());
        }
        super.recieveStateRequest(stateRequest);
    }

    @Override
    public void periodic() {
        StateTable.logObject("Climber/Outputs", outputs);
        climberIO.update();
        switch (activeStateRequest.getStateRequestType()) {
            case UP:
                controlVoltage(ClimberConstants.climberUpPosition.getRadians(), outputs.motorRotation.getRadians());
                if (Math.abs(ClimberConstants.climberUpPosition.getRadians()-outputs.motorRotation.getRadians()) < ClimberConstants.positionTolerance.getRadians())
                    activeStateRequest.updateStatus(StateRequestStatus.FULFILLED);
                break;
            case DOWN:
                controlVoltage(ClimberConstants.climberDownPosition.getRadians(), outputs.motorRotation.getRadians());
                if (Math.abs(ClimberConstants.climberDownPosition.getRadians()-outputs.motorRotation.getRadians()) < ClimberConstants.positionTolerance.getRadians())
                    activeStateRequest.updateStatus(StateRequestStatus.FULFILLED);
                break;
            case MANUAL_OVERRIDE:
                climberIO.setRotorVoltage(ClimberConstants.manualControlVoltage * manualOverrideDirection);
            default:
                break;
        }
    }

    private void controlVoltage(double targetPosition, double currentPosition) {
        double delta = targetPosition - currentPosition;
        double voltage = 0;

        if (delta > 20) voltage = 4;
        else if (delta > 10) voltage = 2;
        else if (delta > 2) voltage = 1;
        else if (delta < -2) voltage = -1;
        else if (delta < -10) voltage = -2;
        else if (delta < -20) voltage = -4;
        else controlPID(targetPosition);
        if (voltage != 0) climberIO.setRotorVoltage(voltage);
    }

    private void controlPID(double targetPosition) {
        climberIO.setTargetPosition(Rotation2d.fromRadians(targetPosition));
    }

    public static ClimberSubStateManager getInstance() {
        return (ClimberSubStateManager) MainStateManager.getInstance().resolveSubStateManager(ClimberStateRequest.class);
    }

    public void manualUp() {
        manualOverrideDirection = 1;
    }
    public void manualDown() {
        manualOverrideDirection = -1;
    }
    public void manualStop() {
        manualOverrideDirection = 0;
    }
}
