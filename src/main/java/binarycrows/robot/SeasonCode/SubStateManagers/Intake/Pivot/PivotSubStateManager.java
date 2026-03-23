package binarycrows.robot.SeasonCode.SubStateManagers.Intake.Pivot;

import org.littletonrobotics.junction.Logger;

import binarycrows.robot.MainStateManager;
import binarycrows.robot.StateRequest;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.Enums.StateRequestStatus;
import binarycrows.robot.SeasonCode.Constants.IntakeConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Intake.Pivot.PivotIO.PivotOutputs;
import binarycrows.robot.SeasonCode.SubStateManagers.Intake.Rollers.IntakeRollersStateRequest;
import binarycrows.robot.Utils.LoggingUtils;
import binarycrows.robot.Utils.Tuning.RuntimeTunableValue;

public class PivotSubStateManager extends SubStateManager<PivotStateRequest>  {
    
    private Pivot pivot;
    private PivotOutputs outputs;

    private RuntimeTunableValue pivotTargetPosition;

    private int manualDirection = 0; // 1=up 0=none 2=down

    private boolean needsToStartRollers = false;

    public PivotSubStateManager() {
        super(new StateRequest<PivotStateRequest>(PivotStateRequest.UP, StateRequestPriority.NORMAL));

        outputs = new PivotOutputs();

        pivot = new Pivot(outputs);

        pivotTargetPosition = new RuntimeTunableValue("Tuning/Pivot/TargetPosition", 2);
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

        if (stateRequest == this.activeStateRequest) {
            switch (this.activeStateRequest.getStateRequestType()) {
                case DOWN:
                    System.out.println("NEEDS TO START ROLLERS");
                    needsToStartRollers = true;
                    break;
                default:
                    needsToStartRollers = false;
                    break;
            }
        }
    }

    @Override
    public void periodic() {

        pivot.update();

        LoggingUtils.logObject("Pivot/Outputs", outputs);
        // Yes, this is probably the best way to control the pivot...
        double voltage = 0;
        boolean runRaisedPID = false;
        double delta = 0;

        switch (this.activeStateRequest.getStateRequestType()) {
            case DOWN:
            System.out.println(outputs.encoderRotation.minus(IntakeConstants.Pivot.intakeRollerActivateThreshold).getDegrees());
            System.out.println(needsToStartRollers);
                if (needsToStartRollers && outputs.encoderRotation.getRotations() > IntakeConstants.Pivot.intakeRollerActivateThreshold.getRotations()) {
                    needsToStartRollers = false;
                    System.out.println("START ROLLERS");
                    new StateRequest<>(IntakeRollersStateRequest.INTAKING, StateRequestPriority.NORMAL).dispatchSelf();;
                }
                delta = IntakeConstants.Pivot.pivotDownPosition.minus(outputs.encoderRotation).getDegrees();
                if (delta > 80) voltage = 1.5;
                else if (delta > 45) voltage = 0.75;
                else if (delta > 20) voltage = 0.1;
                else if (delta > 5) voltage = 0.05;
                else voltage = 0;
                this.activeStateRequest.updateStatus(StateRequestStatus.FULFILLED);
                break;
            case RAISED:
                delta = IntakeConstants.Pivot.pivotRaisedPosition.minus(outputs.encoderRotation).getDegrees();
                if (delta > 25) voltage = 0.5;
                else if (delta > 10) voltage = 0.15;
                else if (delta > 5) voltage = 0.01;
                else if (delta < -25) voltage = -1.5;
                else if (delta < -10) voltage = -.9;
                else if (delta < -5) voltage = -.85;
                else voltage = -.8;
                this.activeStateRequest.updateStatus(StateRequestStatus.FULFILLED);
                //else runRaisedPID = true;
                break;
            case UP:
                delta = IntakeConstants.Pivot.pivotUpPosition.minus(outputs.encoderRotation).getDegrees();
                if (delta < -80) voltage = -1.5;
                else if (delta < -45) voltage = -0.75;
                else if (delta < -25) voltage = -0.3;
                else if (delta < -5) voltage = -0.1;
                else voltage = 0;
                this.activeStateRequest.updateStatus(StateRequestStatus.FULFILLED);
                break;
            case MANUAL_OVERRIDE:
                voltage = manualDirection * IntakeConstants.Pivot.manualVoltage + outputs.encoderRotation.getCos() * IntakeConstants.Pivot.manualVoltageFF;
                break;
        }

        Logger.recordOutput("Intake/Pivot/PositionDelta", delta);

        if (runRaisedPID) pivot.setPIDTarget(IntakeConstants.Pivot.pivotRaisedPosition);
        else pivot.setVoltage(voltage);

        
    }

    public static PivotSubStateManager getInstance() {
        return (PivotSubStateManager) MainStateManager.getInstance().resolveSubStateManager(PivotStateRequest.class);
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
