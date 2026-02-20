package binarycrows.robot.SeasonCode.SubStateManagers.Intake.Pivot;

import binarycrows.robot.MainStateManager;
import binarycrows.robot.StateRequest;
import binarycrows.robot.StateTable;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.SeasonCode.Constants.IntakeConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Intake.Pivot.PivotIO.PivotOutputs;
import binarycrows.robot.Utils.Tuning.RuntimeTunableValue;

public class PivotSubStateManager extends SubStateManager<PivotStateRequest>  {
    
    private Pivot pivot;
    private PivotOutputs outputs;

    private RuntimeTunableValue pivotTargetPosition;

    public PivotSubStateManager() {
        super();

        outputs = new PivotOutputs();

        pivot = new Pivot(outputs);

        pivotTargetPosition = new RuntimeTunableValue("Tuning/Pivot/TargetPosition", 2);

        super.defaultState = new StateRequest<PivotStateRequest>(PivotStateRequest.UP, StateRequestPriority.NORMAL);
    }

    @Override
    public void recieveStateRequest(StateRequest<PivotStateRequest> request) {
        super.recieveStateRequest(request);
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


    //TODO: Implement
    public Runnable manualStop() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'manualStop'");
    }

    public Runnable manualDown() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'manualDown'");
    }

    public Runnable manualUp() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'manualUp'");
    }
}
