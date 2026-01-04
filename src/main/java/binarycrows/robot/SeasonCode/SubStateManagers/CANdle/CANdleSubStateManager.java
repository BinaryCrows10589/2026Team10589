package binarycrows.robot.SeasonCode.SubStateManagers.CANdle;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import binarycrows.robot.StateRequest;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestStatus;
import binarycrows.robot.SeasonCode.Constants.CANIDs;
import binarycrows.robot.SeasonCode.Constants.CANdleConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveStateRequest;

public class CANdleSubStateManager extends SubStateManager<CANdleStateRequest> {

    private CANdle candle;

    public CANdleSubStateManager() {
        super();
        candle = new CANdle(CANIDs.CANdle);
    }

    @Override
    public void recieveStateRequest(StateRequest request) {
        super.recieveStateRequest(request);
    }

    @Override
    public void periodic() {
        super.periodic();
        if (this.activeStateRequest.getStatus() == StateRequestStatus.PENDING) {
            switch (this.activeStateRequest.getStateRequestType()) {
                case RED:
                    setColor(new RGBWColor(255, 0, 0));
                    break;
                case BLUE:
                    setColor(new RGBWColor(0, 0, 255));
                    break;
                case GREEN:
                    setColor(new RGBWColor(0, 255, 0));
                    break;
                default:
                    break;
            }
            this.activeStateRequest.updateStatus(StateRequestStatus.FULFILLED);
        }
    }

    public void setColor(RGBWColor color) {
        candle.setControl(new SolidColor(0, CANdleConstants.numberOfLEDs).withColor(color));

    }

    @Override
    public Class<CANdleStateRequest> getStateRequestType() {
        return CANdleStateRequest.class;
    }

    @Override
    public String toString() {
        return "CANdle SubState Manager";
    }
}
