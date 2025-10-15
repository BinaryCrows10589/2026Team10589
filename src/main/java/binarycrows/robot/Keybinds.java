package binarycrows.robot;

import binarycrows.robot.SeasonCode.Subsystems.Elevator.ElevatorStateRequest;
import binarycrows.robot.SeasonCode.Subsystems.SwerveDrive.DriveStateRequest;
import binarycrows.robot.Utils.StateRequestUtils;
import binarycrows.robot.Utils.Gamepad.GenericGamepad;
import binarycrows.robot.Utils.Gamepad.XboxGamepad;

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
    }
}
