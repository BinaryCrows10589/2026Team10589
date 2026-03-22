package binarycrows.robot.SeasonCode.SubStateManagers.Turret;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import binarycrows.robot.MainStateManager;
import binarycrows.robot.StateRequest;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.Constants.TurretConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Shooting.ShootingSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Turret.TurretIO.TurretOutputs;
import binarycrows.robot.Utils.LoggingUtils;
import edu.wpi.first.math.geometry.Rotation2d;

public class TurretSubStateManager extends SubStateManager<TurretStateRequest> {

    private Turret turret;
    private TurretOutputs outputs;


    private int manualDirection = 0;

    private Rotation2d targetPosition;

    private Supplier<Boolean> shooting;
    private Supplier<Double> shootingTurretAngleRad;
    private Supplier<Boolean> doAim;

    private ArrayList<Double> voltageToVelocityRadPerSecTable = new ArrayList<>();
    private ArrayList<Double> voltageToVelocityVoltageTable = new ArrayList<>();
    private double voltageIncrement = 0.2;
    private double startVoltage = 0;
    private double endVoltage = 8;
    private int framesPerIncrement = 25;
    private int frameCounter = 0;
    private double voltageCounter = 0;

    public TurretSubStateManager() {
        super(new StateRequest<TurretStateRequest>(TurretStateRequest.SHOOT_ON_THE_MOVE, StateRequestPriority.NORMAL));

        outputs = new TurretOutputs();

        turret = new Turret(MetaConstants.isReal ? new TurretTalonFX(outputs) : new TurretSim(outputs));

        

    }

    @Override
    public void setupSuppliers() {
        shooting = ShootingSubStateManager.getInstance()::getShooting;
        shootingTurretAngleRad = ShootingSubStateManager.getInstance()::getTurretAngleRad;
        doAim = ShootingSubStateManager.getInstance()::getDoAim;
    }

    @Override
    public void recieveStateRequest(StateRequest<TurretStateRequest> request) {
        super.recieveStateRequest(request);

        if (activeStateRequest == request) {

            if (activeStateRequest.getStateRequestType() == TurretStateRequest.CONSTRUCT_VOLTAGE_TABLE) {
                frameCounter = 0;
                voltageCounter = 0;
            }

        }
    }

    @Override
    public void periodic() {
        
        LoggingUtils.logObject("Turret/Outputs", outputs);
        turret.update();
        if (targetPosition == null) targetPosition = outputs.encoderRotation;

        switch (activeStateRequest.getStateRequestType()) {
            case MANUAL_OVERRIDE:
                turret.setTurretVoltage(TurretConstants.manualVoltage * (manualDirection));
                break;
            case SHOOT_ON_THE_MOVE:
                if (doAim.get()) turret.setTargetAngle(Rotation2d.fromRadians(shootingTurretAngleRad.get()), shooting.get() && !turret.getHasWrapped());
                Logger.recordOutput("Turret/ShootOnTheMoveTargetAngleRad", shootingTurretAngleRad.get());
                break;
            case CONSTRUCT_VOLTAGE_TABLE:
                
                if (frameCounter > framesPerIncrement){
                    voltageToVelocityRadPerSecTable.add(outputs.turretRotationalVelocityRadPerSec);
                    voltageToVelocityVoltageTable.add(voltageCounter);
                    frameCounter = 0;
                    voltageCounter += voltageIncrement;
                }
                frameCounter++;
                if (voltageCounter > endVoltage) {
                    turret.setTurretVoltage(0);
                } else {
                    turret.setTurretVoltage(voltageCounter);

                }
                Logger.recordOutput("Turret/Table/RotVelRadPerSec", convertLogArray(voltageToVelocityRadPerSecTable.toArray()));
                Logger.recordOutput("Turret/Table/RotVelVoltage", convertLogArray(voltageToVelocityVoltageTable.toArray()));
                break;
        }
         

    }

    private static double[] convertLogArray(Object[] array) {
        double[] v = new double[array.length];
        int i = 0;
        for(Object item : array) {
            v[i++] = (double) item;
        }
        return v;
    }
    private static double[] convertLogArray(ArrayList<Double> array) {
        double[] v = new double[array.size()];
        int i = 0;
        for(Object item : array) {
            v[i++] = (double) item;
        }
        return v;
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
