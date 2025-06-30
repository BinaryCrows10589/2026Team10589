package binarycrows.robot.Utils.Gamepad;

import binarycrows.robot.MainStateManager;
import binarycrows.robot.StateRequest;
import edu.wpi.first.wpilibj.GenericHID;

@SuppressWarnings("rawtypes")
public class GenericGamepad {


    /*
     * Each index is a button index on the gamepad.
     * Each index has an array with onPress at 0 and onRelease at 1
     */
    protected Runnable[][] buttonMap;

    protected GenericHID gamepad;

    public GenericGamepad(int portID, int numButtons) {
        this.gamepad = new GenericHID(portID);
        buttonMap = new Runnable[numButtons][2];

    }

    public void periodic() {
        for(int i = 0; i < buttonMap.length; i++) {
            if(this.gamepad.getRawButtonPressed(i+1)) {
                if(this.buttonMap[i][0] != null)
                    this.buttonMap[i][0].run();
            }
            if(this.gamepad.getRawButtonReleased(i+1)) {
                if(this.buttonMap[i][1] != null)
                    this.buttonMap[i][1].run();
            }
        }
    }

    public void onPress(int buttonIndex, StateRequest onPress, boolean returnOnRelease) {
        onPress(
            buttonIndex, 
            () -> {
                MainStateManager.getInstance().dispatchStateRequest(onPress);
            }
        );

        if (returnOnRelease) {
            onRelease(buttonIndex, () -> {
                MainStateManager.getInstance().requestReturnToDefaultState(onPress.getSubStateManager());
            }
            );
        }
    }

    public void onPress(int buttonIndex, Runnable onPress) {
        buttonMap[buttonIndex][0] = onPress;
    }

    public void onRelease(int buttonIndex, StateRequest onRelease) {
        onRelease(
            buttonIndex, () -> {
                MainStateManager.getInstance().dispatchStateRequest(onRelease);
            }
        );
    }
    public void onRelease(int buttonIndex, Runnable onRelease) {
        buttonMap[buttonIndex][1] = onRelease;
    }

    public double getAxis(int axis) {
        return this.gamepad.getRawAxis(axis);
    }
    public double getPOV(int pov) {
        return this.gamepad.getPOV(pov);
    }

}
