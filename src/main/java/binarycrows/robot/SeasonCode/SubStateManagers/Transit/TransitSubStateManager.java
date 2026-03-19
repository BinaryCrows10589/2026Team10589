package binarycrows.robot.SeasonCode.SubStateManagers.Transit;

import java.util.function.Supplier;

import binarycrows.robot.MainStateManager;
import binarycrows.robot.StateRequest;
import binarycrows.robot.StateTable;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.SeasonCode.Constants.TransitConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Flywheel.FlywheelSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Shooting.ShootingSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Transit.SensorsIO.SensorsOutputs;
import binarycrows.robot.SeasonCode.SubStateManagers.Transit.TransitIO.TransitOutputs;

public class TransitSubStateManager  extends SubStateManager<TransitStateRequest> {
    
    private TransitIO transitIO;
    private TransitOutputs transitOutputs;

    private SensorsIO sensorsIO;
    private SensorsOutputs sensorOutputs;

    private int manualDirection = 0; // 1=forward, 0=off, -1=reverse

    private Supplier<Boolean> shooting;

    private Supplier<Double> flywheelVoltageSupplier;

    public TransitSubStateManager() {
        super(new StateRequest<TransitStateRequest>(TransitStateRequest.SHOOTER, StateRequestPriority.NORMAL));

        transitOutputs = new TransitOutputs();
        sensorOutputs = new SensorsOutputs();

        transitIO = new TransitTalonFXS(transitOutputs);
        sensorsIO = new SensorsPWF(sensorOutputs);

        
    }


    @Override
    public void setupSuppliers() {
        shooting = ShootingSubStateManager.getInstance()::getShooting;
        flywheelVoltageSupplier = FlywheelSubStateManager.getInstance()::getVoltage;

    }

    @Override
    public void recieveStateRequest(StateRequest<TransitStateRequest> request) {
        super.recieveStateRequest(request);
    }

    public boolean getOutgoingFuelSensorTripped() {
        return sensorOutputs.outgoingFuelTripped;
    }

    @Override
    public void periodic() {
        StateTable.logObject("Transit/Outputs", transitOutputs);
        StateTable.logObject("TransitSensors/Outputs", sensorOutputs);

        transitIO.update();
        sensorsIO.update();

        switch (activeStateRequest.getStateRequestType()) {
            case POWERED:
                transitIO.setLatitudinalVoltage(TransitConstants.standardLatitudinalMotorVoltage);
                transitIO.setLongitudinalVoltage(TransitConstants.standardLongitudinalMotorVoltage);
                transitIO.setInAndUpVoltage(TransitConstants.standardInAndUpMotorVoltage);
                break;
            case UNPOWERED:
                transitIO.setLatitudinalVoltage(0);
                transitIO.setLongitudinalVoltage(0);
                transitIO.setInAndUpVoltage(0);
                break;
            case REVERSE:
                transitIO.setLatitudinalVoltage(-TransitConstants.maxLatitudinalMotorVoltage);
                transitIO.setLongitudinalVoltage(-TransitConstants.maxLongitudinalMotorVoltage);
                transitIO.setInAndUpVoltage(-TransitConstants.maxInAndUpMotorVoltage);
                break;
            case MANUAL_OVERRIDE:
                transitIO.setLatitudinalVoltage(TransitConstants.standardLatitudinalMotorVoltage * manualDirection);
                transitIO.setLongitudinalVoltage(TransitConstants.standardLongitudinalMotorVoltage * manualDirection);
                transitIO.setInAndUpVoltage(TransitConstants.standardInAndUpMotorVoltage * manualDirection);
                break;
            case SHOOTER:
                if (shooting.get()) {
                    double flywheelVoltage = flywheelVoltageSupplier.get();
                    transitIO.setLatitudinalVoltage(TransitConstants.standardLatitudinalMotorVoltagePercent * flywheelVoltage);
                    transitIO.setLongitudinalVoltage(TransitConstants.standardLongitudinalMotorVoltagePercent * flywheelVoltage);
                    transitIO.setInAndUpVoltage(TransitConstants.standardInAndUpMotorVoltagePercent * flywheelVoltage);
                } else {
                    transitIO.setLatitudinalVoltage(0);
                    transitIO.setLongitudinalVoltage(0);
                    transitIO.setInAndUpVoltage(0);
                }
                break;
            case INDEX:
                if (sensorOutputs.outgoingFuelReading >= TransitConstants.Sensors.outgoingFuelIndexingDistance) {
                    transitIO.setLatitudinalVoltage(2);
                    transitIO.setLongitudinalVoltage(4);
                    transitIO.setInAndUpVoltage(4); // TODO: Make constants somewhere...
                } else {
                    transitIO.setLatitudinalVoltage(0);
                    transitIO.setLongitudinalVoltage(0);
                    transitIO.setInAndUpVoltage(0);
                    new StateRequest<>(TransitStateRequest.SHOOTER, StateRequestPriority.NORMAL).dispatchSelf();
                }
        }
        
    }

    public String toString() {
        return "TransitSubStateManager";
    }

    public static TransitSubStateManager getInstance() {
        return (TransitSubStateManager) MainStateManager.getInstance().resolveSubStateManager(TransitStateRequest.class);
    }

    public void manualForward() {
        manualDirection = 1;
    }
    public void manualReverse() {
        manualDirection = -1;
    }
    public void manualOff() {
        manualDirection = 0;
    }

}
