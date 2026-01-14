package binarycrows.robot.SeasonCode.SubStateManagers.CANdle;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;

import binarycrows.robot.StateRequest;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestStatus;
import binarycrows.robot.SeasonCode.Constants.CANIDs;
import binarycrows.robot.SeasonCode.Constants.CANdleConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveStateRequest;

public class CANdleSubStateManager extends SubStateManager<CANdleStateRequest> {

    private CANdle candle;
    private CANdleConfiguration candleConfiguration;

    public CANdleSubStateManager() {
        super();
        init();
    }

    @Override
    public void recieveStateRequest(StateRequest request) {
        super.recieveStateRequest(request);
    }

    /**
     * This configures the CANdle
     */
    public void init() {
        this.candle = new CANdle(CANIDs.CANivore.CANdle);
        this.candleConfiguration = new CANdleConfiguration();
        this.candleConfiguration.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Enabled;
        this.candleConfiguration.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.KeepRunning;
        this.candleConfiguration.LED.StripType = StripTypeValue.RGB;
        this.candleConfiguration.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.Modulated;
        this.candle.getConfigurator().apply(candleConfiguration);
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
                case SHOOT_OUT_OF_RANGE:
                    setColorRange(0, 10, new RGBWColor(255, 0, 0));
                    break;
                case SHOOT_CLOSE_TO_RANGE:
                    setColorRange(0, 10, new RGBWColor(255, 255, 0));
                    break;
                case SHOOT_IN_RANGE:
                    setColorRange(0, 10, new RGBWColor(0, 255, 0));
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

    public void setColorRange(int start, int end, RGBWColor color) {
        candle.setControl(new SolidColor(start, end).withColor(color));
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
