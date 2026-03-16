package binarycrows.robot.SeasonCode.Utils;

import binarycrows.robot.Utils.Gamepad.GenericGamepad;

public class ButtonBoard extends GenericGamepad {

    public static class ButtonBoardButtons {
        public static class BB1 {
            public static final int turretManualRight = 0;
            public static final int extra1 = 1;
            public static final int manualHoodSwitch = 2;
            public static final int intakeDown = 3;
            public static final int manualClimberDown = 4;
            public static final int manualClimberUp = 5;
            public static final int climberDown = 6;
            public static final int climberUp = 7;
            public static final int extra2 = 8;
            public static final int climbLeft = 9;
            public static final int hoodManualUp = 10;
            public static final int climbRight = 11;
            public static final int intakeManualUp = 12;
            public static final int manualIntakeSwitch = 13;
            public static final int manualClimberSwitch = 14;
            public static final int manualTransitSwitch = 15;
            public static final int flywheelReverse = 16;
            public static final int manualTurretSwitch = 17;
        }
        public static class BB2 {
            public static final int hoodManualDown = 0;
            public static final int turretManualLeft = 1;
            public static final int hoodForceRetract = 2;
            public static final int climbCenterRight = 3;
            public static final int intakeWheelForceReverse = 4;
            public static final int intakeUp = 5;
            public static final int extra3 = 6;
            public static final int intakeManualDown = 7;
            public static final int forceShoot = 8;
            public static final int transitManualReverse = 9;
            public static final int intakeWheelToggle = 10;
            public static final int shoot = 11;
            public static final int increaseShooterFeedForward = 12;
            public static final int decreaseShooterFeedForward = 13;
            public static final int transitManualForward = 14;
            public static final int climbCenterLeft = 15;
            public static final int extra6 = 16;
            public static final int extra4 = 17;
            public static final int intakeRaised = 18;
            public static final int hoodForceUp = 19;
        }
    }


    public ButtonBoard(int portID, int numButtons) {
        super(portID, numButtons);
    }


}
