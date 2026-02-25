package binarycrows.robot.SeasonCode.SubStateManagers.Flywheel;

import binarycrows.robot.MainStateManager;
import binarycrows.robot.StateRequest;
import binarycrows.robot.StateTable;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.SeasonCode.Constants.FlywheelConstants;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Flywheel.FlywheelIO.FlywheelOutputs;
import binarycrows.robot.SeasonCode.Utils.Shooting;
import edu.wpi.first.math.MathUtil;

public class FlywheelSubStateManager extends SubStateManager<FlywheelStateRequest> {
    
    private FlywheelIO flywheelIO;
    private FlywheelOutputs outputs;


    private double flywheelVoltageFF = FlywheelConstants.baseShooterFF;

    public FlywheelSubStateManager() {
        super(new StateRequest<FlywheelStateRequest>(FlywheelStateRequest.SHOOT_ON_THE_MOVE, StateRequestPriority.NORMAL));

        outputs = new FlywheelOutputs();

        flywheelIO = MetaConstants.isReal ? new FlywheelTalonFX(outputs) : new FlywheelSim(outputs);

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
            case SHOOT_ON_THE_MOVE:
                if (Shooting.isShooting) { // Run at voltage
                    flywheelIO.setRotorVoltage(MathUtil.clamp(Shooting.flywheelVoltage + flywheelVoltageFF, 0, FlywheelConstants.maxMotorVoltage));
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
        }
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
