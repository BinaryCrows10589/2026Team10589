package binarycrows.robot.Utils.Gamepad;

import java.util.HashMap;
import java.util.Map;

import binarycrows.robot.Utils.AutoLogged;
import binarycrows.robot.Utils.AutoMappedProperties;
import binarycrows.robot.Utils.ConversionUtils;

public class XboxGamepad extends GenericGamepad {

    public static class XboxGamepadID {
        public static int a = 0;
        public static int b = 1;
        public static int x = 2;
        public static int y = 3;
        public static int left_bumper = 4;
        public static int right_bumper = 5;
        public static int select = 6;
        public static int start = 7;
        public static int left_stick_button = 8;
        public static int right_stick_button = 9;

        public static int left_stick_x = 0;
        public static int left_stick_y = 1;
        public static int left_trigger = 2;
        public static int right_trigger = 3;
        public static int right_stick_x = 4;
        public static int right_stick_y = 5;

        public static int dpad = 0;
    }
    public static AutoMappedProperties<XboxGamepadID> xboxGamepadID = new AutoMappedProperties<XboxGamepadID>(XboxGamepadID.class);

    public class XboxGamepadInputs extends AutoLogged {
        public XboxGamepadInputs(String path) {
            updatePath(path);
        }

        public void updateButtonByID(int id, boolean value) {
            try {
                XboxGamepadInputs.class.getField(xboxGamepadID.IDToName(id)).set(this, value);
            } catch (IllegalArgumentException | IllegalAccessException | NoSuchFieldException | SecurityException e) {
                e.printStackTrace();
            }
        }
        public void updateAxisByID(int id, int value) {
            try {
                XboxGamepadInputs.class.getField(xboxGamepadID.IDToName(id)).set(this, value);
            } catch (IllegalArgumentException | IllegalAccessException | NoSuchFieldException | SecurityException e) {
                e.printStackTrace();
            }
        }
        public void updateDPad(int value) {
            dpad = value;
        }

        public boolean a;
        public boolean b;
        public boolean x;
        public boolean y;
        public boolean left_bumper;
        public boolean right_bumper;
        public boolean select;
        public boolean start;
        public boolean left_stick;
        public boolean right_stick;
        public float left_stick_x;
        public float left_stick_y;
        public float left_trigger;
        public float right_trigger;
        public float right_stick_x;
        public float right_stick_y;
        public int dpad;
    }


    public XboxGamepadInputs inputs;

    public XboxGamepad(String loggingPath, int portID) {
        super(portID, 0);
        inputs = new XboxGamepadInputs(loggingPath);
    }

    @Override
    public void periodic() {
        boolean inputsDirty = false;
        for(int i = 0; i < buttonMap.length; i++) {
            if(this.gamepad.getRawButtonPressed(i+1)) {
                this.inputs.updateButtonByID(i, true);
                inputsDirty = true;
                if(this.buttonMap[i][0] != null)
                    this.buttonMap[i][0].run();
            }
            if(this.gamepad.getRawButtonReleased(i+1)) {
                this.inputs.updateButtonByID(i, false);
                inputsDirty = true;
                if(this.buttonMap[i][1] != null)
                    this.buttonMap[i][1].run();
            }
        }
        
        if (inputsDirty) inputs.logToStateTable();
    }


}
