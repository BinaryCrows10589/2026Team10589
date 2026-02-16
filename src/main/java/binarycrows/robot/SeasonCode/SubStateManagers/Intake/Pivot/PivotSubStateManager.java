package binarycrows.robot.SeasonCode.SubStateManagers.Intake.Pivot;

import binarycrows.robot.MainStateManager;
import binarycrows.robot.StateRequest;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.SeasonCode.Constants.IntakeConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Intake.Pivot.PivotIO.PivotOutputs;
import binarycrows.robot.Utils.LogIOInputs;

public class PivotSubStateManager extends SubStateManager<PivotStateRequest>  {
    
    private Pivot pivot;
    private PivotOutputs outputs;

    //private RuntimeTunableValue pivotTargetPosition;
    //private RuntimeTunableValue pivotUsePID;

    public PivotSubStateManager() {
        super();

        pivot = new Pivot(outputs);

        //pivotTargetPosition = new RuntimeTunableValue("Tuning/Pivot/TargetPosition", 0.0);
        //pivotUsePID = new RuntimeTunableValue("Tuning/Pivot/UsePIDCatch", false);

        super.defaultState = new StateRequest<PivotStateRequest>(PivotStateRequest.UP, StateRequestPriority.NORMAL);
    }

    @Override
    public void recieveStateRequest(StateRequest<PivotStateRequest> request) {
        super.recieveStateRequest(request);
    }

    @Override
    public void periodic() {
        
        LogIOInputs.logObjectToStateTable(outputs, "Pivot/Outputs");
        // Yes, this is probably the best way to control the pivot...
        double voltage = 0;
        boolean runRaisedPID = false;
        double delta;

        switch (this.activeStateRequest.getStateRequestType()) {
            case UP:
                delta = IntakeConstants.Pivot.pivotDownPosition.minus(outputs.encoderRotation).getDegrees();
                if (delta > 80) voltage = 7;
                else if (delta > 45) voltage = 3;
                else if (delta > 25) voltage = 2;
                else voltage = 0.25;
                break;
            case RAISED:
                delta = IntakeConstants.Pivot.pivotRaisedPosition.minus(outputs.encoderRotation).getDegrees();
                if (delta > 25) voltage = 4;
                else if (delta > 10) voltage = 2;
                else if (delta < 10) voltage = -2;
                else if (delta < 25) voltage = -4;
                else runRaisedPID = true;

                break;
            case DOWN:
                delta = IntakeConstants.Pivot.pivotUpPosition.minus(outputs.encoderRotation).getDegrees();
                if (delta > -80) voltage = -7;
                else if (delta > -45) voltage = -3;
                else if (delta > -25) voltage = -2;
                else voltage = -0.25;
                break;
        }

        if (runRaisedPID) pivot.setPIDTarget(IntakeConstants.Pivot.pivotRaisedPosition);
        else pivot.setVoltage(voltage);

        
        pivot.update();
    }

    public static PivotSubStateManager getInstance() {
        return (PivotSubStateManager) MainStateManager.getInstance().resolveSubStateManager(PivotSubStateManager.class);
    }
    
    public String toString() {
        return "Pivot SubStateManager";
    }
}
