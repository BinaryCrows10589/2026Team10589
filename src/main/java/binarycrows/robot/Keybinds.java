package binarycrows.robot;

import binarycrows.robot.SeasonCode.Constants.ControlConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveSubStateManager;
import binarycrows.robot.Utils.ConversionUtils;
import binarycrows.robot.Utils.StateRequestUtils;
import binarycrows.robot.Utils.Gamepad.GenericGamepad;
import binarycrows.robot.Utils.Gamepad.XboxGamepad;
import edu.wpi.first.math.MathUtil;

public class Keybinds {
    public static GenericGamepad driverController = new XboxGamepad(0);

    public static void periodic() {
        driverController.periodic();
    }

    public static void createKeybinds() {

        /*
        // A -> Basement
        driverController.onPress(
            XboxGamepad.XboxGamepadID.a, 
            StateRequestUtils.createStateRequestRunnable(ElevatorStateRequest.BASEMENT)
        );

        // B -> L1
        driverController.onPress(
            XboxGamepad.XboxGamepadID.a, 
            StateRequestUtils.createStateRequestRunnable(ElevatorStateRequest.CORAL_L1)
        );

        // X -> L4
        driverController.onPress(
            XboxGamepad.XboxGamepadID.a, 
            StateRequestUtils.createStateRequestRunnable(ElevatorStateRequest.CORAL_L4)
        );

        // Y -> Funnel
        driverController.onPress(
            XboxGamepad.XboxGamepadID.a, 
            StateRequestUtils.createStateRequestRunnable(ElevatorStateRequest.FUNNEL)
        );
         */ 
        //1 - 0.1 = 0.9
         //0.5 / 0.9 = (0-1)


        driverController.onPress(
            XboxGamepad.XboxGamepadID.a, 
            StateRequestUtils.createStateRequestRunnable(DriveStateRequest.CONSTRUCT_VOLTAGE_TABLE)
        );
        driverController.onPress(
            XboxGamepad.XboxGamepadID.b, 
            StateRequestUtils.createStateRequestRunnable(DriveStateRequest.DISABLE)
        );
        driverController.onPress(
            XboxGamepad.XboxGamepadID.x, 
            StateRequestUtils.createStateRequestRunnable(DriveStateRequest.TELEOP_DRIVE)
        );
        driverController.onPress(
            XboxGamepad.XboxGamepadID.y, 
            StateRequestUtils.createStateRequestRunnable(DriveStateRequest.DRIVE_DISTANCE_TEST)
        );
        driverController.onPress(
            XboxGamepad.XboxGamepadID.left_bumper, 
            DriveSubStateManager.getInstance()::resetRobotPose
        );
    }

    public static double[] getTranslation() {
        double xTranslation = -(driverController.getAxis(XboxGamepad.XboxGamepadID.left_stick_y));
        double yTranslation = -(driverController.getAxis(XboxGamepad.XboxGamepadID.left_stick_x));

        double magnitude = Math.sqrt(xTranslation * xTranslation + yTranslation * yTranslation);
        
        double deadbandMagnitude = MathUtil.applyDeadband(magnitude, ControlConstants.driveControllerDeadband);
        if (deadbandMagnitude == 0) return new double[] {0,0};

        double xTranslationDeadbanded = (xTranslation / magnitude) * deadbandMagnitude;
        double yTranslationDeadbanded = (yTranslation / magnitude) * deadbandMagnitude;

        return new double[] {xTranslationDeadbanded * ControlConstants.maxSpeedFraction, yTranslationDeadbanded * ControlConstants.maxSpeedFraction};
    }
    public static double getRotation() {
        return -ConversionUtils.applyDeadband(driverController.getAxis(XboxGamepad.XboxGamepadID.right_stick_x), ControlConstants.driveControllerDeadband) * ControlConstants.maxSpeedFraction;
    }
    public static double getThrottle() {
        return driverController.getAxis(XboxGamepad.XboxGamepadID.right_trigger) * 0.2;
    }
    private static int axisLock = 0; // 0 - None, 1 - X, 2 - Y
    public static int getAxisLock() {
        int pov = driverController.getPOV(XboxGamepad.XboxGamepadID.dpad);
        if (pov != -1) {
            if (pov == 0) {
                axisLock = 2;
            }
            else if (pov == 90) {
                axisLock = 1;
            }
            else {
                axisLock = 0;
            }
        }
        return axisLock;
    }
}
