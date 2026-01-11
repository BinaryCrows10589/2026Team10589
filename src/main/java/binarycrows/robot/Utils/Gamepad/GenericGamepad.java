package binarycrows.robot.Utils.Gamepad;

import binarycrows.robot.MainStateManager;
import binarycrows.robot.StateRequest;
import binarycrows.robot.Utils.LogIOInputs;
import edu.wpi.first.wpilibj.GenericHID;

@SuppressWarnings("rawtypes")
public class GenericGamepad {


    /*
     * Each index is a button index on the gamepad.
     * Each index has an array with onPress at 0 and onRelease at 1
     */
    protected Runnable[][] buttonMap;
    protected boolean[] lastFrameReadings;

    protected GenericHID gamepad;

    public GenericGamepad(int portID, int numButtons) {
        this.gamepad = new GenericHID(portID);
        buttonMap = new Runnable[numButtons][2];
        lastFrameReadings = new boolean[numButtons];

    }

    public void periodic() {
        for(int i = 0; i < buttonMap.length; i++) {
            if(this.gamepad.getRawButtonPressed(i+1)) {
                
                if(this.buttonMap[i][0] != null && !this.lastFrameReadings[i])
                    this.buttonMap[i][0].run();

                this.lastFrameReadings[i] = true;
            }
            if(this.gamepad.getRawButtonReleased(i+1) && this.lastFrameReadings[i]) {
                if(this.buttonMap[i][1] != null)
                    this.buttonMap[i][1].run();

                    this.lastFrameReadings[i] = false;
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

    public void onPress(int buttonIndex, Runnable onPress, Runnable onRelease) {
        buttonMap[buttonIndex][0] = onPress;
        buttonMap[buttonIndex][1] = onPress;
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
        double value = this.gamepad.getRawAxis(axis);
        LogIOInputs.logToStateTable(value, "Gamepad/" + gamepad.getPort() + "/Axis/" + axis);
        return value;
    }
    public int getPOV(int pov) {
        return this.gamepad.getPOV(pov);
    }

}
