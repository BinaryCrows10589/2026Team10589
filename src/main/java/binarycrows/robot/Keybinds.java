package binarycrows.robot;

import binarycrows.robot.SeasonCode.Constants.ControlConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.Climber.ClimberStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.Flywheel.FlywheelStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.Flywheel.FlywheelSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Hood.HoodStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.Hood.HoodSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Intake.Pivot.PivotStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.Intake.Pivot.PivotSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Intake.Rollers.IntakeRollersStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.Shooting.ShootingStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Transit.TransitStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.Transit.TransitSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Turret.TurretStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.Turret.TurretSubStateManager;
import binarycrows.robot.SeasonCode.Utils.ButtonBoard;
import binarycrows.robot.SeasonCode.Utils.ButtonBoard.ButtonBoardButtons;
import binarycrows.robot.Utils.ConversionUtils;
import binarycrows.robot.Utils.StateRequestUtils;
import binarycrows.robot.Utils.Gamepad.GenericGamepad;
import binarycrows.robot.Utils.Gamepad.XboxGamepad;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class Keybinds {
    public static GenericGamepad driverController = new XboxGamepad(0);
    public static GenericGamepad buttonBoard1 = new ButtonBoard(1, 18);
    public static GenericGamepad buttonBoard2 = new ButtonBoard(2, 20);
    public static GenericGamepad testController = new XboxGamepad(3);



    public static void periodic() {
        driverController.periodic();
        buttonBoard1.periodic();
        testController.periodic();

    }

    public static Rotation2d getTestAngle() {
        double x = testController.getAxis(XboxGamepad.XboxGamepadID.left_stick_x);
        double y = testController.getAxis(XboxGamepad.XboxGamepadID.left_stick_y);

        if (Math.abs(x) < .5 && Math.abs(y) < .5) return null;

        return Rotation2d.fromRadians(Math.atan2(x, -y));
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

        // Test Controller

        testController.onPress(XboxGamepad.XboxGamepadID.x, 
        StateRequestUtils.createStateRequestRunnable(ShootingStateRequest.FORCE_SHOOT),
        StateRequestUtils.createStateRequestRunnable(ShootingStateRequest.STANDBY)
        );

        testController.onPress(XboxGamepad.XboxGamepadID.y, 
        StateRequestUtils.createStateRequestRunnable(IntakeRollersStateRequest.INTAKING),
        StateRequestUtils.createStateRequestRunnable(IntakeRollersStateRequest.OFF));
        

        // Button Board

        // Intake
        buttonBoard1.onPress(ButtonBoardButtons.BB1.intakeDown, 
        StateRequestUtils.createDualStateRequestRunnable(PivotStateRequest.DOWN, IntakeRollersStateRequest.INTAKING));
        buttonBoard2.onPress(ButtonBoardButtons.BB2.intakeRaised, 
        StateRequestUtils.createDualStateRequestRunnable(PivotStateRequest.RAISED, IntakeRollersStateRequest.REVERSE));
        buttonBoard2.onPress(ButtonBoardButtons.BB2.intakeUp, 
        StateRequestUtils.createDualStateRequestRunnable(PivotStateRequest.UP, IntakeRollersStateRequest.OFF));

        buttonBoard2.onPress(ButtonBoardButtons.BB2.intakeManualDown, 
        PivotSubStateManager.getInstance()::manualDown,
        PivotSubStateManager.getInstance()::manualStop);
        buttonBoard1.onPress(ButtonBoardButtons.BB1.intakeManualUp, 
        PivotSubStateManager.getInstance()::manualUp,
        PivotSubStateManager.getInstance()::manualStop);
        
        buttonBoard2.onPress(ButtonBoardButtons.BB2.intakeWheelToggle, 
        StateRequestUtils.createStateRequestRunnable(IntakeRollersStateRequest.INVERT));

        // Transit
        buttonBoard2.onPress(ButtonBoardButtons.BB2.transitManualForward, 
        TransitSubStateManager.getInstance()::manualForward);
        buttonBoard2.onPress(ButtonBoardButtons.BB2.transitManualReverse, 
        TransitSubStateManager.getInstance()::manualReverse);

        // Turret
        buttonBoard2.onPress(ButtonBoardButtons.BB2.turretManualLeft, 
        TurretSubStateManager.getInstance()::manualLeft, 
        TurretSubStateManager.getInstance()::manualStop);
        buttonBoard1.onPress(ButtonBoardButtons.BB1.turretManualRight, 
        TurretSubStateManager.getInstance()::manualRight, 
        TurretSubStateManager.getInstance()::manualStop);

        // Hood
        buttonBoard1.onPress(ButtonBoardButtons.BB1.hoodManualUp, 
        HoodSubStateManager.getInstance()::manualUp,
        HoodSubStateManager.getInstance()::manualStop);
        buttonBoard2.onPress(ButtonBoardButtons.BB2.hoodManualDown, 
        HoodSubStateManager.getInstance()::manualDown,
        HoodSubStateManager.getInstance()::manualStop);
        buttonBoard2.onPress(ButtonBoardButtons.BB2.hoodForceRetract, 
        StateRequestUtils.createStateRequestRunnable(HoodStateRequest.RETRACTED));
        buttonBoard2.onPress(ButtonBoardButtons.BB2.hoodForceUp, 
        StateRequestUtils.createStateRequestRunnable(HoodStateRequest.SHOOT_ON_THE_MOVE));

        // Shooting
        buttonBoard2.onPress(ButtonBoardButtons.BB2.shoot, 
            StateRequestUtils.createStateRequestRunnable(ShootingStateRequest.SHOOT),
            StateRequestUtils.createStateRequestRunnable(ShootingStateRequest.STANDBY));
        buttonBoard2.onPress(ButtonBoardButtons.BB2.forceShoot,  
            StateRequestUtils.createStateRequestRunnable(ShootingStateRequest.FORCE_SHOOT),
            StateRequestUtils.createStateRequestRunnable(ShootingStateRequest.STANDBY));
        buttonBoard2.onPress(ButtonBoardButtons.BB2.increaseShooterFeedForward, FlywheelSubStateManager.getInstance()::increaseShooterFF);
        buttonBoard2.onPress(ButtonBoardButtons.BB2.decreaseShooterFeedForward, FlywheelSubStateManager.getInstance()::decreaseShooterFF);
        buttonBoard1.onPress(ButtonBoardButtons.BB1.flywheelReverse, 
        StateRequestUtils.createStateRequestRunnable(FlywheelStateRequest.REVERSE), 
        StateRequestUtils.createStateRequestRunnable(FlywheelStateRequest.SHOOT_ON_THE_MOVE));
        
        // Climber
        /* 
        buttonBoard1.onPress(ButtonBoardButtons.BB1.climbLeft, Climbing::climbLeft, Climbing::cancelClimb);
        buttonBoard1.onPress(ButtonBoardButtons.BB1.climbRight, Climbing::climbRight, Climbing::cancelClimb);
        buttonBoard2.onPress(ButtonBoardButtons.BB2.climbCenterLeft, Climbing::climbCenterLeft, Climbing::cancelClimb);
        buttonBoard2.onPress(ButtonBoardButtons.BB2.climbCenterRight, Climbing::climbCenterRight, Climbing::cancelClimb);
        buttonBoard1.onPress(ButtonBoardButtons.BB1.climberUp, StateRequestUtils.createStateRequestRunnable(ClimberStateRequest.UP));
        buttonBoard1.onPress(ButtonBoardButtons.BB1.climberDown, StateRequestUtils.createStateRequestRunnable(ClimberStateRequest.DOWN));
        buttonBoard1.onPress(ButtonBoardButtons.BB1.manualClimberUp, 
        ClimberSubStateManager.getInstance()::manualUp, 
        ClimberSubStateManager.getInstance()::manualStop);
        buttonBoard1.onPress(ButtonBoardButtons.BB1.manualClimberDown, 
        ClimberSubStateManager.getInstance()::manualDown, 
        ClimberSubStateManager.getInstance()::manualStop);*/

        // Switches
        buttonBoard1.onPress(ButtonBoardButtons.BB1.manualIntakeSwitch,
        StateRequestUtils.createStateRequestRunnable(PivotStateRequest.MANUAL_OVERRIDE),
        StateRequestUtils.createStateRequestRunnable(PivotStateRequest.RESTORE_CLOSEST));
        buttonBoard1.onPress(ButtonBoardButtons.BB1.manualTransitSwitch, 
        StateRequestUtils.createStateRequestRunnable(TransitStateRequest.MANUAL_OVERRIDE),
        StateRequestUtils.createStateRequestRunnable(TransitStateRequest.SHOOTER));
        buttonBoard1.onPress(ButtonBoardButtons.BB1.manualTurretSwitch, 
        StateRequestUtils.createStateRequestRunnable(TurretStateRequest.MANUAL_OVERRIDE),
        StateRequestUtils.createStateRequestRunnable(TurretStateRequest.SHOOT_ON_THE_MOVE));
        buttonBoard1.onPress(ButtonBoardButtons.BB1.manualHoodSwitch,  
        StateRequestUtils.createStateRequestRunnable(HoodStateRequest.MANUAL_OVERRIDE),
        StateRequestUtils.createStateRequestRunnable(HoodStateRequest.SHOOT_ON_THE_MOVE));
        buttonBoard1.onPress(ButtonBoardButtons.BB1.manualClimberSwitch, 
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
