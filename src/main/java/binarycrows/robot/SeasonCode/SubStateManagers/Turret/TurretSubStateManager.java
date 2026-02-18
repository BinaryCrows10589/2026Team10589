package binarycrows.robot.SeasonCode.SubStateManagers.Turret;

import binarycrows.robot.MainStateManager;
import binarycrows.robot.StateRequest;
import binarycrows.robot.StateTable;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.Enums.StateRequestStatus;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Turret.TurretIO.TurretOutputs;
import binarycrows.robot.Utils.Tuning.RuntimeTunableValue;
import edu.wpi.first.math.geometry.Rotation2d;

public class TurretSubStateManager extends SubStateManager<TurretStateRequest> {

    private Turret turret;
    private TurretOutputs outputs;

    private RuntimeTunableValue turretTargetPosition;
    private RuntimeTunableValue isShooting;

    


    public TurretSubStateManager() {
        super();

        outputs = new TurretOutputs();

        turret = new Turret(MetaConstants.isReal ? new TurretIOTalonFX(outputs) : new TurretIOSim(outputs));

        turretTargetPosition = new RuntimeTunableValue("Tuning/Turret/TargetPosition", 0.0);
        isShooting = new RuntimeTunableValue("Tuning/Turret/IsShooting", false);

        super.defaultState = new StateRequest<TurretStateRequest>(TurretStateRequest.SHOOT_ON_THE_MOVE, StateRequestPriority.NORMAL);
    }

    @Override
    public void recieveStateRequest(StateRequest<TurretStateRequest> request) {
        request.updateStatus(StateRequestStatus.REJECTED);
    }

    @Override
    public void periodic() {
        
        StateTable.logObject("Turret/Outputs", outputs);
        turret.update();
        // Must be ran after turret update so we get new encoder values
        turret.setTargetAngle(Rotation2d.fromDegrees((double)turretTargetPosition.getValue()), (boolean)isShooting.getValue());

    }

    public static TurretSubStateManager getInstance() {
        return (TurretSubStateManager) MainStateManager.getInstance().resolveSubStateManager(TurretStateRequest.class);
    } 
    
    public String toString() {
        return "Turret SubStateManager";
    }
}
