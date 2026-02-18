package binarycrows.robot.SeasonCode.SubStateManagers.Climber;

import binarycrows.robot.StateRequest;
import binarycrows.robot.StateTable;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.SeasonCode.Constants.ClimberConstants;
import binarycrows.robot.SeasonCode.Constants.FlywheelConstants;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Climber.ClimberIO.ClimberOutputs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class ClimberSubStateManager extends SubStateManager<ClimberStateRequest> {
    
    private ClimberIO climberIO;
    private ClimberOutputs outputs;

    public ClimberSubStateManager() {
        super();
        outputs = new ClimberOutputs();

        climberIO = new ClimberTalonFX(outputs);

        super.defaultState = new StateRequest<ClimberStateRequest>(ClimberStateRequest.DOWN, StateRequestPriority.NORMAL);
    }

    @Override
    public void recieveStateRequest(StateRequest<ClimberStateRequest> stateRequest) {
        super.recieveStateRequest(stateRequest);
    }

    @Override
    public void periodic() {
        StateTable.logObject("Climber/Outputs", outputs);
        climberIO.update();
        switch (activeStateRequest.getStateRequestType()) {
            case UP:
                controlVoltage(ClimberConstants.climberUpPosition.getRadians(), outputs.motorRotation.getRadians());
                break;
            case DOWN:
                controlVoltage(ClimberConstants.climberDownPosition.getRadians(), outputs.motorRotation.getRadians());
                break;
        }
    }

    private void controlVoltage(double targetPosition, double currentPosition) {
        double delta = targetPosition - currentPosition;
        double voltage = 0;

        if (delta > 20) voltage = 4;
        else if (delta > 10) voltage = 2;
        else if (delta > 2) voltage = 1;
        else if (delta < -2) voltage = -1;
        else if (delta < -10) voltage = -2;
        else if (delta < -20) voltage = -4;
        else controlPID(targetPosition);
        if (voltage != 0) climberIO.setRotorVoltage(voltage);
    }

    private void controlPID(double targetPosition) {
        climberIO.setTargetPosition(Rotation2d.fromRadians(targetPosition));
    }
}
