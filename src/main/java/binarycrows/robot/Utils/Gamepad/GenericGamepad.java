package binarycrows.robot.Utils.Gamepad;

import binarycrows.robot.MainStateManager;
import binarycrows.robot.StateRequest;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.NullStateRequestType;

@SuppressWarnings("rawtypes")
public class GenericGamepad extends SubStateManager<NullStateRequestType> {

    /*
     * Each index is a button index on the gamepad.
     * Each index has an array with onPress at 0 and onRelease at 1
     */
    private Runnable[][] buttonMap;

    public GenericGamepad(int portID, int numButtons) {
        super();
        buttonMap = new Runnable[numButtons][2];
    }

    @Override
    public void periodic() {

    }

    public void onPress(int buttonIndex, StateRequest onPress, boolean returnOnRelease) {
        buttonMap[buttonIndex][0] = () -> {
            MainStateManager.getInstance().dispatchStateRequest(onPress);
        };
        if (returnOnRelease) {
            buttonMap[buttonIndex][1] = () -> {
                MainStateManager.getInstance().requestReturnToDefaultState(onPress.getSubStateManager());
            };
        }
    }

    public void onRelease(int buttonIndex, StateRequest onRelease) {
        buttonMap[buttonIndex][1] = () -> {
            MainStateManager.getInstance().dispatchStateRequest(onRelease);
        };
    }

}
