package binarycrows.robot.SeasonCode.SubStateManagers.Flywheel;

import binarycrows.robot.MainStateManager;
import binarycrows.robot.StateRequest;
import binarycrows.robot.StateTable;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.SeasonCode.Constants.FlywheelConstants;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Flywheel.FlywheelIO.FlywheelOutputs;
import binarycrows.robot.SeasonCode.SubStateManagers.Hood.HoodStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.Hood.HoodSubStateManager;
import edu.wpi.first.math.MathUtil;

public class FlywheelSubStateManager extends SubStateManager<FlywheelStateRequest> {
    
    private FlywheelIO flywheelIO;
    private FlywheelOutputs outputs;

    private double flywheelVoltage = 0;

    private double flywheelVoltageFF = FlywheelConstants.baseShooterFF;

    public FlywheelSubStateManager() {
        super();

        outputs = new FlywheelOutputs();

        flywheelIO = MetaConstants.isReal ? new FlywheelTalonFX(outputs) : new FlywheelSim(outputs);

        super.defaultState = new StateRequest<FlywheelStateRequest>(FlywheelStateRequest.SHOOT_ON_THE_MOVE, StateRequestPriority.NORMAL);
    }

    @Override
    public void recieveStateRequest(StateRequest<FlywheelStateRequest> stateRequest) {
        super.recieveStateRequest(stateRequest);
    }

    @Override
    public void periodic() {
        StateTable.logObject("Flywheel/Outputs", outputs);
        flywheelIO.update();

        switch (activeStateRequest.getStateRequestType()) {
            //TODO: You are going to want a way to mvoe the flywheel backword in case of a jam
            case SHOOT_ON_THE_MOVE:
                flywheelIO.setRotorVoltage(MathUtil.clamp(flywheelVoltage + flywheelVoltageFF, 0, FlywheelConstants.maxMotorVoltage));
                break;
            case IDLE:
                if (outputs.leftMotorVelocityRPS < FlywheelConstants.idleMinVelocityRPS) {
                    flywheelIO.setRotorVoltage(FlywheelConstants.idleRecoveryVoltage);
                } else {
                    flywheelIO.setRotorVoltage(0);
                }
                break;
            case OFF:
                flywheelIO.setRotorVoltage(0);
                break;
        }
    }

    public void setFlywheelVoltage(double flywheelVoltage) {
        this.flywheelVoltage = flywheelVoltage;
    }

    public static FlywheelSubStateManager getInstance() {
        return (FlywheelSubStateManager) MainStateManager.getInstance().resolveSubStateManager(FlywheelStateRequest.class);
    }


    public void increaseShooterFF() {
        flywheelVoltageFF += FlywheelConstants.shooterFFIncrement;
    }

    public void decreaseShooterFF() {
        flywheelVoltageFF -= FlywheelConstants.shooterFFIncrement;
    }
}
