package binarycrows.robot.SeasonCode.SubStateManagers.Hood;

import binarycrows.robot.MainStateManager;
import binarycrows.robot.StateRequest;
import binarycrows.robot.StateTable;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.SeasonCode.Constants.HoodConstants;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Hood.HoodIO.HoodOutputs;
import binarycrows.robot.SeasonCode.SubStateManagers.Turret.TurretStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.Turret.TurretSubStateManager;
import binarycrows.robot.Utils.Tuning.RuntimeTunableValue;
import edu.wpi.first.math.geometry.Rotation2d;

public class HoodSubStateManager extends SubStateManager<HoodStateRequest> {

    //TODO: when making hood class, add: 
    // standard pids; 
    // voltage if statements w/ integrated pid catch; 
    // and evil turret control mode

    //TODO: All need gravity feedforward, which is tuned value * sin of angle of hood relative to the horizontal or whatever it is
    
    private Hood hood;
    private HoodOutputs outputs;

    private RuntimeTunableValue hoodTargetPosition;
    private RuntimeTunableValue mustRetract;

    public HoodSubStateManager() {
        super();

        outputs = new HoodOutputs();

        hood = new Hood(MetaConstants.isReal ? 
        HoodConstants.useIntegratedPID ? new HoodTalonFXSIntegratedPID(outputs) : new HoodTalonFXSWPILibPID(outputs)
        : new HoodSim(outputs));

        hoodTargetPosition = new RuntimeTunableValue("Tuning/Hood/TargetPosition", 0.0);
        mustRetract = new RuntimeTunableValue("Tuning/Hood/MustRetract", false);

        super.defaultState = new StateRequest<HoodStateRequest>(HoodStateRequest.RETRACTED, StateRequestPriority.NORMAL);
    }

    @Override
    public void recieveStateRequest(StateRequest<HoodStateRequest> request) {
        super.recieveStateRequest(request);
    }

    @Override
    public void periodic() {
        StateTable.logObject("Hood/Outputs", outputs);
        hood.update();
        controlVoltage();
        
    }

    private void controlPID() {
        hood.setPIDTarget(Rotation2d.fromDegrees(getTargetPosition()));
    }
    private void controlVoltage() {
        double voltage = 0;
        double delta = getTargetPosition() - outputs.hoodRotation.getDegrees();

        if (delta > 20) voltage = 0.02;
        else if (delta > 5) voltage = 0.002;
        else if (delta < -5) voltage = -0.002;
        else if (delta < -20) voltage = -0.02;
        else controlPID();

        if (voltage != 0) hood.setVoltage(voltage);
    }
    private void controlLikeTurret() {
        hood.setTargetLikeTurret(Rotation2d.fromDegrees(getTargetPosition()));
    }

    private double getTargetPosition() {
        if ((boolean)mustRetract.getValue()) {
            return 0.0;
        }
        return (double)hoodTargetPosition.getValue();
    }

    public static HoodSubStateManager getInstance() {
        return (HoodSubStateManager) MainStateManager.getInstance().resolveSubStateManager(HoodStateRequest.class);
    }

    //TODO: Implement

    public Runnable manualStop() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'manualStop'");
    }

    public Runnable manualUp() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'manualUp'");
    }

    public Runnable manualDown() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'manualDown'");
    } 
}
