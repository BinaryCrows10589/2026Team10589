package binarycrows.robot.SeasonCode.SubStateManagers.Flywheel;

import java.util.ArrayList;
import java.util.function.Supplier;

import binarycrows.robot.MainStateManager;
import binarycrows.robot.StateRequest;
import binarycrows.robot.StateTable;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.SeasonCode.Constants.FlywheelConstants;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Flywheel.FlywheelIO.FlywheelOutputs;
import binarycrows.robot.SeasonCode.SubStateManagers.Shooting.ShootingSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Turret.TurretStateRequest;
import edu.wpi.first.math.MathUtil;

public class FlywheelSubStateManager extends SubStateManager<FlywheelStateRequest> {
    
    private FlywheelIO flywheelIO;
    private FlywheelOutputs outputs;


    private double flywheelVoltageFF = FlywheelConstants.baseShooterFF;

    private Supplier<Boolean> isShootingSupplier;
    private Supplier<Double> flywheelVoltageSupplier;

    private ArrayList<Double> voltageToVelocityRotPerSecTable = new ArrayList<>();
    private ArrayList<Double> voltageToVelocityVoltageTable = new ArrayList<>();
    private double voltageIncrement = 0.2;
    private double endVoltage = 8;
    private int framesPerIncrement = 25;
    private int frameCounter = 0;
    private double voltageCounter = 0;

    public FlywheelSubStateManager() {
        super(new StateRequest<FlywheelStateRequest>(FlywheelStateRequest.SHOOT_ON_THE_MOVE, StateRequestPriority.NORMAL));

        outputs = new FlywheelOutputs();

        flywheelIO = MetaConstants.isReal ? new FlywheelTalonFX(outputs) : new FlywheelSim(outputs);

        
    }

    public void setupSuppliers() {
        isShootingSupplier = ShootingSubStateManager.getInstance()::getShooting;
        flywheelVoltageSupplier = ShootingSubStateManager.getInstance()::getFlywheelVoltage;
    }

    @Override
    public void recieveStateRequest(StateRequest<FlywheelStateRequest> stateRequest) {
        super.recieveStateRequest(stateRequest);

        if (activeStateRequest == stateRequest) {

            if (activeStateRequest.getStateRequestType() == FlywheelStateRequest.CONSTRUCT_VOLTAGE_TABLE) {
                frameCounter = 0;
                voltageCounter = 0;
            }

        }
    }

    @Override
    public void periodic() {
        StateTable.logObject("Flywheel/Outputs", outputs);
        flywheelIO.update();

        switch (activeStateRequest.getStateRequestType()) {
            case SHOOT_ON_THE_MOVE:
                if (isShootingSupplier.get()) { // Run at voltage
                    flywheelIO.setRotorVoltage(FlywheelConstants.motorShootingVoltage);//MathUtil.clamp(flywheelVoltageSupplier.get() + flywheelVoltageFF, 0, FlywheelConstants.maxMotorVoltage));
                } else { // Run idle
                    if (outputs.leftMotorVelocityRPS < FlywheelConstants.idleMinVelocityRPS) {
                        flywheelIO.setRotorVoltage(FlywheelConstants.idleRecoveryVoltage);
                    } else {
                        flywheelIO.setRotorVoltage(0);
                    }
                }
                break;
            case IDLE:
                if (outputs.leftMotorVelocityRPS < FlywheelConstants.idleMinVelocityRPS) {
                    flywheelIO.setRotorVoltage(FlywheelConstants.idleRecoveryVoltage);
                } else {
                    flywheelIO.setRotorVoltage(0);
                }
                break;
            case REVERSE:
                flywheelIO.setRotorVoltage(FlywheelConstants.reverseVoltage);
                break;
            case OFF:
                flywheelIO.setRotorVoltage(0);
                break;
            case CONSTRUCT_VOLTAGE_TABLE:
                
                if (frameCounter > framesPerIncrement){
                    voltageToVelocityRotPerSecTable.add(outputs.leftMotorVelocityRPS);
                    voltageToVelocityVoltageTable.add(voltageCounter);
                    frameCounter = 0;
                    voltageCounter += voltageIncrement;
                }
                frameCounter++;
                if (voltageCounter > endVoltage) {
                    flywheelIO.setRotorVoltage(0);
                    (new StateRequest<>(FlywheelStateRequest.OFF, StateRequestPriority.NORMAL)).dispatchSelf();
                } else {
                    flywheelIO.setRotorVoltage(voltageCounter);

                }
                StateTable.log("Turret/Table/RotVelRotPerSec", convertLogArray(voltageToVelocityRotPerSecTable));
                StateTable.log("Turret/Table/RotVelVoltage", convertLogArray(voltageToVelocityVoltageTable.toArray()));
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

    public static FlywheelSubStateManager getInstance() {
        return (FlywheelSubStateManager) MainStateManager.getInstance().resolveSubStateManager(FlywheelStateRequest.class);
    }

    public double getRPM() {
        return outputs.leftMotorVelocityRPS * 60.0 * FlywheelConstants.gearRatio;
    }


    public void increaseShooterFF() {
        flywheelVoltageFF += FlywheelConstants.shooterFFIncrement;
    }

    public void decreaseShooterFF() {
        flywheelVoltageFF -= FlywheelConstants.shooterFFIncrement;
    }
}
