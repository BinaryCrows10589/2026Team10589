package binarycrows.robot.SeasonCode.SubStateManagers.Hood;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import binarycrows.robot.MainStateManager;
import binarycrows.robot.StateRequest;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.SeasonCode.Constants.HoodConstants;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Hood.HoodIO.HoodOutputs;
import binarycrows.robot.SeasonCode.SubStateManagers.Shooting.ShootingSubStateManager;
import binarycrows.robot.Utils.LoggingUtils;
import edu.wpi.first.math.geometry.Rotation2d;

public class HoodSubStateManager extends SubStateManager<HoodStateRequest> {
    
    private Hood hood;
    private HoodOutputs outputs;


    private Rotation2d manualOverrideTargetRotation = HoodConstants.hoodStartingPosition;

    private Rotation2d targetPosition = HoodConstants.hoodStartingPosition;

    private int manualDirection = 0; // 1=up, 0=none, -1=down

    private Supplier<Boolean> closeToTrenchSupplier;
    private Supplier<Double> hoodAngleSupplier;

    public HoodSubStateManager() {
        super(new StateRequest<HoodStateRequest>(HoodStateRequest.SHOOT_ON_THE_MOVE, StateRequestPriority.NORMAL));

        outputs = new HoodOutputs();

        hood = new Hood(MetaConstants.isReal ? 
        HoodConstants.useIntegratedPID ? new HoodTalonFXSIntegratedPID(outputs) : new HoodTalonFXSWPILibPID(outputs)
        : new HoodSim(outputs));
    }

    @Override
    public void recieveStateRequest(StateRequest<HoodStateRequest> request) {
        super.recieveStateRequest(request);
        if (activeStateRequest.getStateRequestType() == HoodStateRequest.MANUAL_OVERRIDE) {
            manualOverrideTargetRotation = outputs.hoodRotation;
        }

    }

    @Override
    public void setupSuppliers() {
        closeToTrenchSupplier = ShootingSubStateManager.getInstance()::getIsCloseToTrench;
        hoodAngleSupplier = ShootingSubStateManager.getInstance()::getHoodAngleRad;

    }

    @Override
    public void periodic() {
        LoggingUtils.logObject("Hood/Outputs", outputs);
        switch (activeStateRequest.getStateRequestType()) {
            case MANUAL_OVERRIDE:
                manualOverrideTargetRotation = manualOverrideTargetRotation.plus(HoodConstants.manualAngleIncrement.times(manualDirection));
                targetPosition = manualOverrideTargetRotation;
                break;
            case SHOOT_ON_THE_MOVE:
                if (closeToTrenchSupplier.get()) targetPosition = Rotation2d.kZero;
                else targetPosition = Rotation2d.fromRadians(hoodAngleSupplier.get());
                break;
            case RETRACTED:
                targetPosition = Rotation2d.kZero;
                break;
        }
        hood.update();
        controlVoltage();
        
    }

    private void controlPID() {
        hood.setPIDTarget(targetPosition);
    }
    private void controlVoltage() {
        double voltage = 0;
        boolean isPID = false;
        double delta = targetPosition.getDegrees() - outputs.hoodRotation.getDegrees();
        double velocity = Math.max(Math.abs(outputs.hoodRotationalVelocityRadPerSec), 2);
        if (velocity == 2) velocity = 1;

        Logger.recordOutput("/Hood/ControlDelta", delta);
        if (Math.abs(delta) < 1 && targetPosition.getDegrees() == 0) voltage = 0;
        else if (delta > 20) voltage = 1.5 / velocity;
        else if (delta > 10) voltage = 1 / velocity;
        else if (delta > 7) voltage = 0.9 / velocity;
        else if (delta > 4) voltage = 0.9 / velocity / velocity;
        else if (delta < -20) voltage = -.25 / velocity;
        else if (delta < -10) voltage = -.1 / velocity;
        else if (delta < -7) voltage = 0.1 * velocity/1.5;
        else if (delta < -4) voltage = 0.2 * velocity/1.5;
        else {
            controlPID();
            isPID = true;
        }

        if (!isPID) hood.setVoltage(voltage);
    }
    private void controlLikeTurret() {
        hood.setTargetLikeTurret(targetPosition);
    }

    public static HoodSubStateManager getInstance() {
        return (HoodSubStateManager) MainStateManager.getInstance().resolveSubStateManager(HoodStateRequest.class);
    }

    public double getDeltaRad() {
        return outputs.distanceFromSetpoint.getRadians();
    }


    public void manualStop() {
        manualDirection = 0;
    }

    public void manualUp() {
        manualDirection = 1;
    }

    public void manualDown() {
        manualDirection = -1;
    } 
}
