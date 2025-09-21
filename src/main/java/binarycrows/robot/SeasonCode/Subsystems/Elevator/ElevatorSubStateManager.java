package binarycrows.robot.SeasonCode.Subsystems.Elevator;

import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestStatus;
import binarycrows.robot.SeasonCode.Constants.ElevatorConstants;
import binarycrows.robot.Utils.LogIOInputs;

public class ElevatorSubStateManager extends SubStateManager<ElevatorStateRequest> {

    ElevatorDualKrakenIO elevatorIO;

    public ElevatorSubStateManager() {
        super();
        elevatorIO = new ElevatorDualKrakenIO();
    }

    @Override
    public void periodic() {

        // Resolve pending state request
        if (this.activeStateRequest.getStatus() == StateRequestStatus.PENDING) {
            switch (this.activeStateRequest.getStateRequestType()) {
                default:
                    // Resolve the state request's indicated position or simply do nothing
                    elevatorIO.setDesiredPosition(ElevatorConstants.elevatorPositions.getOrDefault(activeStateRequest, elevatorIO.elevatorRawPosition));
                    break;
            }
            this.activeStateRequest.updateStatus(StateRequestStatus.FULFILLED);

        }
        LogIOInputs.logToStateTable(elevatorIO, "Elevator");

    }
}
