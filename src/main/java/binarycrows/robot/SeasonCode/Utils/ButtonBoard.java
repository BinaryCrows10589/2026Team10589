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
        public static final int flywheelReverse = 18; //TODO: Add to CAD

        // Climber
        public static final int climbLeft = 19;
        public static final int climbRight = 20;
        public static final int climbCenterLeft = 21;
        public static final int climbCenterRight = 22; //TODO: Add to CAD
        public static final int climberUp = 23;
        public static final int climberDown = 24;
        public static final int manualClimberUp = 25;
        public static final int manualClimberDown = 26;
        
        // Extra Buttons
        public static final int extra1 = 27;
        public static final int extra2 = 28;
        public static final int extra3 = 29;
        public static final int extra4 = 30;
        public static final int extra5 = 31;
        public static final int extra6 = 32; //TODO: Add to CAD

        // Switches
        public static final int manualIntakeSwitch = 33;
        public static final int manualTransitSwitch = 34;
        public static final int manualTurretSwitch = 35;
        public static final int manualHoodSwitch = 36;
        public static final int manualClimberSwitch = 37;
    }


    public ButtonBoard(int portID) {
        super(portID, 33);
    }


}
