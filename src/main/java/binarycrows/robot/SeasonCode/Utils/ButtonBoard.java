package binarycrows.robot.SeasonCode.Utils;

import binarycrows.robot.Utils.Gamepad.GenericGamepad;

public class ButtonBoard extends GenericGamepad {

    public static class ButtonBoardButtons {
        // Intake
        public static final int intakeDown = 0;
        public static final int intakeRaised = 1;
        public static final int intakeUp = 2;
        public static final int intakeManualDown = 3;
        public static final int intakeManualUp = 4;
        public static final int intakeWheelToggle = 5;
        public static final int intakeWheelForceReverse = 6;
        
        // Transit
        public static final int transitManualForward = 7;
        public static final int transitManualReverse = 8;
        
        // Turret
        public static final int turretManualLeft = 9;
        public static final int turretManualRight = 10;

        // Hood
        public static final int hoodManualUp = 11;
        public static final int hoodManualDown = 12;
        public static final int hoodForceRetract = 13;

        // Shooting
        public static final int shoot = 14;
        public static final int forceShoot = 15;
        public static final int increaseShooterFF = 16;
        public static final int decreaseShooterFF = 17;

        // Climber
        public static final int climbLeft = 18;
        public static final int climbRight = 19;
        public static final int climbCenterLeft = 20;
        public static final int climbCenterRight = 21; //TODO
        public static final int climberUp = 21;
        public static final int climberDown = 22;
        public static final int manualClimberUp = 23;
        public static final int manualClimberDown = 24;
        
        // Extra Buttons
        public static final int extra1 = 25;
        public static final int extra2 = 26;
        public static final int extra3 = 27;
        public static final int extra4 = 28;
        public static final int extra5 = 29;

        // Switches
        public static final int manualIntakeSwitch = 30;
        public static final int manualTransitSwitch = 31;
        public static final int manualTurretSwitch = 32;
        public static final int manualHoodSwitch = 33;
        public static final int manualClimberSwitch = 34;
    }


    public ButtonBoard(int portID) {
        super(portID, 33);
    }


}
