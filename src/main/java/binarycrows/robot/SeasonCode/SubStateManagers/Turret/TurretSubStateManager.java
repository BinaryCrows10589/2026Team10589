package binarycrows.robot.SeasonCode.SubStateManagers.Turret;

import binarycrows.robot.MainStateManager;
import binarycrows.robot.StateRequest;
import binarycrows.robot.StateTable;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.Enums.StateRequestStatus;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.Constants.TurretConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Turret.TurretIO.TurretOutputs;
import binarycrows.robot.Utils.Tuning.RuntimeTunableValue;
import edu.wpi.first.math.geometry.Rotation2d;

public class TurretSubStateManager extends SubStateManager<TurretStateRequest> {

    private Turret turret;
    private TurretOutputs outputs;

    private Rotation2d shootOnTheMoveTargetPosition;
    private Rotation2d manualTargetPosition;

    private int manualDirection = 0;

    private Rotation2d targetPosition;


    public TurretSubStateManager() {
        super();

        outputs = new TurretOutputs();

        turret = new Turret(MetaConstants.isReal ? new TurretTalonFX(outputs) : new TurretSim(outputs));

        super.defaultState = new StateRequest<TurretStateRequest>(TurretStateRequest.SHOOT_ON_THE_MOVE, StateRequestPriority.NORMAL);
    }

    @Override
    public void recieveStateRequest(StateRequest<TurretStateRequest> request) {
        if (request.getStateRequestType() == TurretStateRequest.MANUAL_OVERRIDE) {
            manualTargetPosition = outputs.turretRotation;
        }
        request.updateStatus(StateRequestStatus.REJECTED);
    }

    @Override
    public void periodic() {
        
        StateTable.logObject("Turret/Outputs", outputs);
        turret.update();
        if (targetPosition == null) targetPosition = outputs.turretRotation;

        switch (activeStateRequest.getStateRequestType()) {
            case MANUAL_OVERRIDE:
                manualTargetPosition = manualTargetPosition.plus(TurretConstants.manualPositionIncrement.times(manualDirection));
                turret.setTargetAngle(manualTargetPosition, true);
                break;
            case SHOOT_ON_THE_MOVE:
                //TODO: make isShooting be an actual thing somewhere else
                turret.setTargetAngle(shootOnTheMoveTargetPosition, isShooting.getValue());;
                break;
        }
         

    }

    public static TurretSubStateManager getInstance() {
        return (TurretSubStateManager) MainStateManager.getInstance().resolveSubStateManager(TurretStateRequest.class);
    } 

    public void putShootOnTheMoveTargetPosition(Rotation2d targetPosition) {
        shootOnTheMoveTargetPosition = targetPosition;
    }
    
    public String toString() {
        return "TurretSubStateManager";
    }


    public void manualLeft() {
        manualDirection = 1;
    }

    public void manualRight() {
        manualDirection = -1;
    }

    public void manualStop() {
        manualDirection = 0;
    }
}
