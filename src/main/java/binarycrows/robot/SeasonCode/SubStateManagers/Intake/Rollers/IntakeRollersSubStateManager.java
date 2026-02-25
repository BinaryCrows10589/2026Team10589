package binarycrows.robot.SeasonCode.SubStateManagers.Intake.Rollers;

import binarycrows.robot.StateRequest;
import binarycrows.robot.StateTable;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.SeasonCode.Constants.IntakeConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Intake.Rollers.IntakeRollersIO.IntakeRollersOutputs;

public class IntakeRollersSubStateManager extends SubStateManager<IntakeRollersStateRequest> {
    
    private IntakeRollersIO intakeRollersIO;
    private IntakeRollersOutputs outputs;

    public IntakeRollersSubStateManager() {
        super(new StateRequest<IntakeRollersStateRequest>(IntakeRollersStateRequest.OFF, StateRequestPriority.NORMAL));

        outputs = new IntakeRollersOutputs();
        intakeRollersIO = new IntakeRollersTalonFXS(outputs);

    }

    @Override
    public void recieveStateRequest(StateRequest<IntakeRollersStateRequest> request) {
        if (request.getStateRequestType() == IntakeRollersStateRequest.INVERT) {
            request = new StateRequest<IntakeRollersStateRequest>(
                (activeStateRequest.getStateRequestType() == IntakeRollersStateRequest.OFF) ? IntakeRollersStateRequest.INTAKING : IntakeRollersStateRequest.OFF, // If we're already off, start intaking. Otherwise, shut off.
                request.getPriority());
        }
        super.recieveStateRequest(request);
    }

    @Override
    public void periodic() {
        StateTable.logObject("Intake/Rollers/Outputs", outputs);
        intakeRollersIO.update();
        switch (activeStateRequest.getStateRequestType()) {
            case INTAKING:
                intakeRollersIO.setRotorVoltage(IntakeConstants.Rollers.intakingMotorVoltage);
                break;
            case OFF:
                intakeRollersIO.setRotorVoltage(0);
                break;
            case REVERSE:
                intakeRollersIO.setRotorVoltage(-IntakeConstants.Rollers.intakingMotorVoltage);
                break;
            default:
                break;
        }
    }

    public String toString() {
        return "IntakeRollersSubStateManager";
    }
}
