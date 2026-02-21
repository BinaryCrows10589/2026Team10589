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
        super();

        outputs = new IntakeRollersOutputs();
        intakeRollersIO = new IntakeRollersTalonFXS(outputs);

        super.defaultState = new StateRequest<IntakeRollersStateRequest>(IntakeRollersStateRequest.OFF, StateRequestPriority.NORMAL);
    }

    @Override
    public void recieveStateRequest(StateRequest<IntakeRollersStateRequest> request) {
        super.recieveStateRequest(request);
    }

    @Override
    public void periodic() {
        StateTable.logObject("Intake/Rollers/Outputs", outputs);
        intakeRollersIO.update();
        // TODO: For voltage to run at you dont necesarry whant it to be the same as max. 
        // This makes the RPS of the motor less constent due to voltage flucuations
        // Lets say the supplied voltage flucatiosn by .1v up and down you are now preventign all the upepr 
        // flucations while allowing all the lower flucations making actually average voltage being 3.95
        // Just use seperate variables.
        switch (activeStateRequest.getStateRequestType()) {
            case INTAKING:
                intakeRollersIO.setRotorVoltage(IntakeConstants.Rollers.maxMotorVoltage);
                break;
            case OFF:
                intakeRollersIO.setRotorVoltage(0);
                break;
            case REVERSE:
                intakeRollersIO.setRotorVoltage(-IntakeConstants.Rollers.maxMotorVoltage);
                break;
        }
    }

    public String toString() {
        return "IntakeRollersSubStateManager";
    }
}
