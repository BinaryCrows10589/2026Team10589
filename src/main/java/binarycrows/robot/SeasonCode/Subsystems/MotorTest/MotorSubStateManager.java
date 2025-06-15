package binarycrows.robot.SeasonCode.Subsystems.MotorTest;

import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestStatus;
import binarycrows.robot.Utils.LogIOInputs;

public class MotorSubStateManager extends SubStateManager<MotorTestStateRequest> {

    MotorTestIO motorIO;

    public MotorSubStateManager() {
        super();
        motorIO = new MotorTestKraken();
    }

    @Override
    public void periodic() {
        if (this.activeStateRequest.getStatus() == StateRequestStatus.PENDING) {
            switch (this.activeStateRequest.getStateRequestType()) {
                case STOPPED:
                    motorIO.setSpeed(0);
                    break;
                case MOVING:
                    motorIO.setSpeed(0.5);
                    break;
                case MOVING_QUITE_FAST:
                    motorIO.setSpeed(1);
                    break;
            }
            this.activeStateRequest.updateStatus(StateRequestStatus.FULFILLED);

            LogIOInputs.logToStateTable(motorIO, "MotorTest");
        }
    }
}
