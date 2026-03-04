package binarycrows.robot.SeasonCode.SubStateManagers.Turret;

import java.util.function.Supplier;

import binarycrows.robot.MainStateManager;
import binarycrows.robot.StateRequest;
import binarycrows.robot.StateTable;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.Constants.TurretConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Shooting.ShootingSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Turret.TurretIO.TurretOutputs;
import edu.wpi.first.math.geometry.Rotation2d;

public class TurretSubStateManager extends SubStateManager<TurretStateRequest> {

    private Turret turret;
    private TurretOutputs outputs;


    private int manualDirection = 0;

    private Rotation2d targetPosition;

    private Supplier<Boolean> shooting;
    private Supplier<Double> shootingTurretAngleRad;

    public TurretSubStateManager() {
        super(new StateRequest<TurretStateRequest>(TurretStateRequest.SHOOT_ON_THE_MOVE, StateRequestPriority.NORMAL));

        outputs = new TurretOutputs();

        turret = new Turret(MetaConstants.isReal ? new TurretTalonFX(outputs) : new TurretSim(outputs));

        shooting = ShootingSubStateManager.getInstance()::getShooting;
        shootingTurretAngleRad = ShootingSubStateManager.getInstance()::getTurretAngleRad;

    }

    @Override
    public void recieveStateRequest(StateRequest<TurretStateRequest> request) {
        super.recieveStateRequest(request);
    }

    @Override
    public void periodic() {
        
        StateTable.logObject("Turret/Outputs", outputs);
        turret.update();
        if (targetPosition == null) targetPosition = outputs.turretRotation;

        switch (activeStateRequest.getStateRequestType()) {
            case MANUAL_OVERRIDE:
                turret.setTurretVoltage(TurretConstants.manualVoltage * (manualDirection));
                break;
            case SHOOT_ON_THE_MOVE:
                turret.setTargetAngle(Rotation2d.fromRadians(shootingTurretAngleRad.get()), shooting.get());
                break;
        }
         

    }

    public static TurretSubStateManager getInstance() {
        return (TurretSubStateManager) MainStateManager.getInstance().resolveSubStateManager(TurretStateRequest.class);
    } 

    
    public String toString() {
        return "TurretSubStateManager";
    }

    public double getDeltaRad() {
        return outputs.distanceFromSetpoint.getRadians();
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
