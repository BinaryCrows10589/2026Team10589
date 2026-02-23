package binarycrows.robot;

import binarycrows.robot.SeasonCode.Constants.ControlConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Climber.ClimberStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.Climber.ClimberSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Flywheel.FlywheelStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.Flywheel.FlywheelSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Hood.HoodStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.Hood.HoodSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Intake.Pivot.PivotStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.Intake.Pivot.PivotSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Intake.Rollers.IntakeRollersStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Transit.TransitStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.Transit.TransitSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Turret.TurretStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.Turret.TurretSubStateManager;
import binarycrows.robot.SeasonCode.Utils.ButtonBoard;
import binarycrows.robot.SeasonCode.Utils.Climbing;
import binarycrows.robot.SeasonCode.Utils.ManualOverrides;
import binarycrows.robot.SeasonCode.Utils.Shooting;
import binarycrows.robot.SeasonCode.Utils.ButtonBoard.ButtonBoardButtons;
import binarycrows.robot.Utils.ConversionUtils;
import binarycrows.robot.Utils.StateRequestUtils;
import binarycrows.robot.Utils.Gamepad.GenericGamepad;
import binarycrows.robot.Utils.Gamepad.XboxGamepad;
import edu.wpi.first.math.MathUtil;

public class Keybinds {
    public static GenericGamepad driverController = new XboxGamepad(0);
    public static GenericGamepad buttonBoard1 = new ButtonBoard(1);


    public static void periodic() {
        driverController.periodic();
        buttonBoard1.periodic();

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
            XboxGamepad.XboxGamepadID.left_bumper, 
            DriveSubStateManager.getInstance()::enableForceRobotRelative,
            DriveSubStateManager.getInstance()::disableForceRobotRelative
        );
        driverController.onPress(
            XboxGamepad.XboxGamepadID.y, 
            DriveSubStateManager.getInstance()::resetRobotPose
        );
        driverController.onPress(
            XboxGamepad.XboxGamepadID.right_bumper, 
            DriveSubStateManager.getInstance()::enableSlowMode,
            DriveSubStateManager.getInstance()::disableSlowMode
        );

        // Button Board

        // Intake
        buttonBoard1.onPress(ButtonBoardButtons.intakeDown, 
        StateRequestUtils.createDualStateRequestRunnable(PivotStateRequest.DOWN, IntakeRollersStateRequest.INTAKING));
        buttonBoard1.onPress(ButtonBoardButtons.intakeRaised, 
        StateRequestUtils.createDualStateRequestRunnable(PivotStateRequest.RAISED, IntakeRollersStateRequest.REVERSE));
        buttonBoard1.onPress(ButtonBoardButtons.intakeUp, 
        StateRequestUtils.createDualStateRequestRunnable(PivotStateRequest.UP, IntakeRollersStateRequest.OFF));

        buttonBoard1.onPress(ButtonBoardButtons.intakeManualDown, 
        PivotSubStateManager.getInstance()::manualDown,
        PivotSubStateManager.getInstance()::manualStop);
        buttonBoard1.onPress(ButtonBoardButtons.intakeManualUp, 
        PivotSubStateManager.getInstance()::manualUp,
        PivotSubStateManager.getInstance()::manualStop);
        
        buttonBoard1.onPress(ButtonBoardButtons.intakeWheelToggle, 
        StateRequestUtils.createStateRequestRunnable(IntakeRollersStateRequest.INVERT));

        // Transit
        buttonBoard1.onPress(ButtonBoardButtons.transitManualForward, 
        TransitSubStateManager.getInstance()::manualForward);
        buttonBoard1.onPress(ButtonBoardButtons.transitManualReverse, 
        TransitSubStateManager.getInstance()::manualReverse);

        // Turret
        buttonBoard1.onPress(ButtonBoardButtons.turretManualLeft, 
        TurretSubStateManager.getInstance()::manualLeft, 
        TurretSubStateManager.getInstance()::manualStop);
        buttonBoard1.onPress(ButtonBoardButtons.turretManualRight, 
        TurretSubStateManager.getInstance()::manualRight, 
        TurretSubStateManager.getInstance()::manualStop);

        // Hood
        buttonBoard1.onPress(ButtonBoardButtons.hoodManualUp, 
        HoodSubStateManager.getInstance()::manualUp,
        HoodSubStateManager.getInstance()::manualStop);
        buttonBoard1.onPress(ButtonBoardButtons.hoodManualDown, 
        HoodSubStateManager.getInstance()::manualDown,
        HoodSubStateManager.getInstance()::manualStop);
        buttonBoard1.onPress(ButtonBoardButtons.hoodForceRetract, 
        StateRequestUtils.createStateRequestRunnable(HoodStateRequest.RETRACTED));

        // Shooting
        buttonBoard1.onPress(ButtonBoardButtons.shoot, 
            () -> {Shooting.isShooting = true;},
            () -> {Shooting.isShooting = false;});
        buttonBoard1.onPress(ButtonBoardButtons.forceShoot,  
            () -> {Shooting.isForceShooting = true;},
            () -> {Shooting.isForceShooting = false;});
        buttonBoard1.onPress(ButtonBoardButtons.increaseShooterFF, FlywheelSubStateManager.getInstance()::increaseShooterFF);
        buttonBoard1.onPress(ButtonBoardButtons.decreaseShooterFF, FlywheelSubStateManager.getInstance()::decreaseShooterFF);
        buttonBoard1.onPress(ButtonBoardButtons.flywheelReverse, 
        StateRequestUtils.createStateRequestRunnable(FlywheelStateRequest.REVERSE), 
        StateRequestUtils.createStateRequestRunnable(FlywheelStateRequest.SHOOT_ON_THE_MOVE));
        
        // Climber
        buttonBoard1.onPress(ButtonBoardButtons.climbLeft, Climbing::climbLeft, Climbing::cancelClimb);
        buttonBoard1.onPress(ButtonBoardButtons.climbRight, Climbing::climbRight, Climbing::cancelClimb);
        buttonBoard1.onPress(ButtonBoardButtons.climbCenterLeft, Climbing::climbCenterLeft, Climbing::cancelClimb);
        buttonBoard1.onPress(ButtonBoardButtons.climbCenterRight, Climbing::climbCenterRight, Climbing::cancelClimb);
        buttonBoard1.onPress(ButtonBoardButtons.climberUp, StateRequestUtils.createStateRequestRunnable(ClimberStateRequest.UP));
        buttonBoard1.onPress(ButtonBoardButtons.climberDown, StateRequestUtils.createStateRequestRunnable(ClimberStateRequest.DOWN));
        buttonBoard1.onPress(ButtonBoardButtons.manualClimberUp, 
        ClimberSubStateManager.getInstance()::manualUp, 
        ClimberSubStateManager.getInstance()::manualStop);
        buttonBoard1.onPress(ButtonBoardButtons.manualClimberDown, 
        ClimberSubStateManager.getInstance()::manualDown, 
        ClimberSubStateManager.getInstance()::manualStop);

        // Switches
        buttonBoard1.onPress(ButtonBoardButtons.manualIntakeSwitch,
        StateRequestUtils.createStateRequestRunnable(PivotStateRequest.MANUAL_OVERRIDE),
        StateRequestUtils.createStateRequestRunnable(PivotStateRequest.RESTORE_CLOSEST));
        buttonBoard1.onPress(ButtonBoardButtons.manualTransitSwitch, 
        StateRequestUtils.createStateRequestRunnable(TransitStateRequest.MANUAL_OVERRIDE),
        StateRequestUtils.createStateRequestRunnable(TransitStateRequest.SHOOTER));
        buttonBoard1.onPress(ButtonBoardButtons.manualTurretSwitch, 
        StateRequestUtils.createStateRequestRunnable(TurretStateRequest.MANUAL_OVERRIDE),
        StateRequestUtils.createStateRequestRunnable(TurretStateRequest.SHOOT_ON_THE_MOVE));
        buttonBoard1.onPress(ButtonBoardButtons.manualHoodSwitch,  
        StateRequestUtils.createStateRequestRunnable(HoodStateRequest.MANUAL_OVERRIDE),
        StateRequestUtils.createStateRequestRunnable(HoodStateRequest.SHOOT_ON_THE_MOVE));
        buttonBoard1.onPress(ButtonBoardButtons.manualClimberSwitch, 
        StateRequestUtils.createStateRequestRunnable(ClimberStateRequest.MANUAL_OVERRIDE),
        StateRequestUtils.createStateRequestRunnable(ClimberStateRequest.RESTORE_CLOSEST));
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
