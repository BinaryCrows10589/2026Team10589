package binarycrows.robot.Utils.Gamepad;

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


    public XboxGamepad(int portID) {
        super(portID, 9);
    }


}
