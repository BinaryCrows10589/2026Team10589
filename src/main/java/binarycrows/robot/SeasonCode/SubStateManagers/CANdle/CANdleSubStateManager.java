package binarycrows.robot.SeasonCode.SubStateManagers.CANdle;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;

import binarycrows.robot.StateRequest;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestPriority;
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

    @SuppressWarnings("rawtypes")
    @Override
    public void recieveStateRequest(StateRequest request) {
        super.recieveStateRequest(request);
    }

    public static void setLEDs(CANdleStateRequest request) {
        (new StateRequest<CANdleStateRequest>(request, StateRequestPriority.NORMAL)).dispatchSelf();
    }

    /**
     * This configures the CANdle
     */
    public void init() {
        this.candle = new CANdle(CANIDs.RIO.CANdle);
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

                case SHOOT_BAD_DISTANCE_TOO_HIGH:
                    setColorRange(CANdleConstants.shooterLEDStart, CANdleConstants.shooterLEDEnd, new RGBWColor(255, 0, 0));
                    break;
                case SHOOT_BAD_DISTANCE_WAY_TOO_HIGH:
                    setStrobeRange(CANdleConstants.shooterLEDStart, CANdleConstants.shooterLEDEnd, new RGBWColor(255, 0, 0));
                    break;
                case SHOOT_BAD_HOOD_DELTA_TOO_HIGH:
                    setColorRange(CANdleConstants.shooterLEDStart, CANdleConstants.shooterLEDEnd, new RGBWColor(255, 127, 127));
                    break;
                case SHOOT_BAD_TURRET_DELTA_TOO_HIGH:
                    setColorRange(CANdleConstants.shooterLEDStart, CANdleConstants.shooterLEDEnd, new RGBWColor(255, 64, 64));
                    break;
                case SHOOT_BAD_VELOCITY_TOO_HIGH:
                    setColorRange(CANdleConstants.shooterLEDStart, CANdleConstants.shooterLEDEnd, new RGBWColor(255, 255, 0));
                    break;
                case SHOOT_BAD_VELOCITY_WAY_TOO_HIGH:
                    setStrobeRange(CANdleConstants.shooterLEDStart, CANdleConstants.shooterLEDEnd, new RGBWColor(255, 255, 0));
                    break;
                case SHOOT_BAD_ACCELERATION_TOO_HIGH:
                    setColorRange(CANdleConstants.shooterLEDStart, CANdleConstants.shooterLEDEnd, new RGBWColor(255, 127, 0));
                    break;
                case SHOOT_BAD_WRONG_SIDE_OF_FIELD:
                    setRainbowRange(CANdleConstants.shooterLEDStart, CANdleConstants.shooterLEDEnd);
                    break;
                case SHOOT_GOOD:
                    setColorRange(CANdleConstants.shooterLEDStart, CANdleConstants.shooterLEDEnd, new RGBWColor(0, 255, 0));
                    break;

                case HOPPER_FULL:
                    setColorRange(CANdleConstants.binLEDStart, CANdleConstants.binLEDEnd, new RGBWColor(0, 255, 0));
                    break;
                
                case HOPPER_PARTIAL:
                    setColorRange(CANdleConstants.binLEDStart, CANdleConstants.binLEDEnd, new RGBWColor(255, 255, 0));
                    break;
                case HOPPER_EMPTY:
                    setColorRange(CANdleConstants.binLEDStart, CANdleConstants.binLEDEnd, new RGBWColor(255, 0, 0));
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

    public void setStrobeRange(int start, int end, RGBWColor color) {
        candle.setControl(new StrobeAnimation(start, end).withColor(color).withFrameRate(15));
    }

    public void setRainbowRange(int start, int end) {
        candle.setControl(new RainbowAnimation(start, end).withFrameRate(30));
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
