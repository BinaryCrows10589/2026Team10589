package binarycrows.robot;

import binarycrows.robot.SeasonCode.Constants.ControlConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveStateRequest;
import binarycrows.robot.Utils.ConversionUtils;
import binarycrows.robot.Utils.StateRequestUtils;
import binarycrows.robot.Utils.Gamepad.GenericGamepad;
import binarycrows.robot.Utils.Gamepad.XboxGamepad;
import edu.wpi.first.wpilibj.RobotBase;

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
    }

    public static double getTranslationX() {
        return -ConversionUtils.applyDeadband(driverController.getAxis(XboxGamepad.XboxGamepadID.left_stick_y), ControlConstants.driveControllerDeadband) * ControlConstants.maxSpeedFraction;
    }
    public static double getTranslationY() {
        return -ConversionUtils.applyDeadband(driverController.getAxis(XboxGamepad.XboxGamepadID.left_stick_x), ControlConstants.driveControllerDeadband) * ControlConstants.maxSpeedFraction;
    }
    public static double getRotation() {
        return -ConversionUtils.applyDeadband(driverController.getAxis(XboxGamepad.XboxGamepadID.right_stick_x), ControlConstants.driveControllerDeadband) * ControlConstants.maxSpeedFraction;
    }
}
