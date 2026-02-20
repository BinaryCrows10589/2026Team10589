package binarycrows.robot.SeasonCode.SubStateManagers.Intake.Pivot;

import binarycrows.robot.MainStateManager;
import binarycrows.robot.StateRequest;
import binarycrows.robot.StateTable;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.SeasonCode.Constants.ClimberConstants;
import binarycrows.robot.SeasonCode.Constants.IntakeConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Climber.ClimberStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.Intake.Pivot.PivotIO.PivotOutputs;
import binarycrows.robot.Utils.Tuning.RuntimeTunableValue;

public class PivotSubStateManager extends SubStateManager<PivotStateRequest>  {
    
    private Pivot pivot;
    private PivotOutputs outputs;

    private RuntimeTunableValue pivotTargetPosition;

    private int manualDirection = 0; // 1=up 0=none 2=down

    public PivotSubStateManager() {
        super();

        outputs = new PivotOutputs();

        pivot = new Pivot(outputs);

        pivotTargetPosition = new RuntimeTunableValue("Tuning/Pivot/TargetPosition", 2);

        super.defaultState = new StateRequest<PivotStateRequest>(PivotStateRequest.UP, StateRequestPriority.NORMAL);
    }

    @Override
    public void recieveStateRequest(StateRequest<PivotStateRequest> stateRequest) {
        if (stateRequest.getStateRequestType() == PivotStateRequest.RESTORE_CLOSEST) {
            double distanceToUpPositionRad = Math.abs(IntakeConstants.Pivot.pivotUpPosition.getRadians()-outputs.encoderRotation.getRadians());
            double distanceToDownPositionRad = Math.abs(IntakeConstants.Pivot.pivotDownPosition.getRadians()-outputs.encoderRotation.getRadians());

            stateRequest = new StateRequest<PivotStateRequest>(
                (distanceToUpPositionRad < distanceToDownPositionRad) ? PivotStateRequest.UP : PivotStateRequest.DOWN, 
            stateRequest.getPriority());
        }
        super.recieveStateRequest(stateRequest);
    }

    @Override
    public void periodic() {

        pivot.update();

        StateTable.logObject("Pivot/Outputs", outputs);
        // Yes, this is probably the best way to control the pivot...
        double voltage = 0;
        boolean runRaisedPID = false;
        double delta = 0;

        System.out.println(pivotTargetPosition.getValue());


        switch (this.activeStateRequest.getStateRequestType()) {
            case DOWN:
                delta = IntakeConstants.Pivot.pivotDownPosition.minus(outputs.encoderRotation).getDegrees();
                if (delta < -80) voltage = -7;
                else if (delta < -45) voltage = -3;
                else if (delta < -25) voltage = -2;
                else if (delta < -5) voltage = -1;
                else voltage = -0.25;
                break;
            case RAISED:
                delta = IntakeConstants.Pivot.pivotRaisedPosition.minus(outputs.encoderRotation).getDegrees();
                if (delta > 25) voltage = 4;
                else if (delta > 10) voltage = 2;
                else if (delta < -10) voltage = -2;
                else if (delta < -25) voltage = -4;
                else runRaisedPID = true;

                break;
            case UP:
                delta = IntakeConstants.Pivot.pivotUpPosition.minus(outputs.encoderRotation).getDegrees();
                if (delta > 80) voltage = 7;
                else if (delta > 45) voltage = 3;
                else if (delta > 25) voltage = 2;
                else if (delta > 5) voltage = 1;
                else voltage = 0.25;
                break;
            case MANUAL_OVERRIDE:
                voltage = manualDirection * IntakeConstants.Pivot.manualVoltage + outputs.encoderRotation.getSin() * IntakeConstants.Pivot.manualVoltageFF; //TODO: make sure Sin here is correct
                break;
        }

        System.out.println("Delta: " + delta);

        if (runRaisedPID) pivot.setPIDTarget(IntakeConstants.Pivot.pivotRaisedPosition);
        else pivot.setVoltage(voltage);

        
    }

    public static PivotSubStateManager getInstance() {
        return (PivotSubStateManager) MainStateManager.getInstance().resolveSubStateManager(PivotSubStateManager.class);
    }
    
    public String toString() {
        return "Pivot SubStateManager";
    }


    public void manualStop() {
        manualDirection = 0;
    }

    public void manualDown() {
        manualDirection = -1;
    }

    public void manualUp() {
        manualDirection = 1;
    }
}
