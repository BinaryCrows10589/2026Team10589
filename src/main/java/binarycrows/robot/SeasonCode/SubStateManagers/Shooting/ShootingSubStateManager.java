package binarycrows.robot.SeasonCode.SubStateManagers.Shooting;

import java.util.Arrays;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import binarycrows.robot.Keybinds;
import binarycrows.robot.MainStateManager;
import binarycrows.robot.Robot;
import binarycrows.robot.StateRequest;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.Enums.StateRequestStatus;
import binarycrows.robot.SeasonCode.Constants.FlywheelConstants;
import binarycrows.robot.SeasonCode.Constants.MetaConstants;
import binarycrows.robot.SeasonCode.Constants.ShootingConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.CANdle.CANdleStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.CANdle.CANdleSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Flywheel.FlywheelSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Hood.HoodSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Transit.TransitSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Turret.TurretSubStateManager;
import binarycrows.robot.Utils.LoggingUtils;
import binarycrows.robot.Utils.Tuning.RuntimeTunablePIDValues;
import binarycrows.robot.Utils.Tuning.RuntimeTunableValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ShootingSubStateManager extends SubStateManager<ShootingStateRequest> {
    private boolean canShoot = false;

    private boolean closeToTrench = false;

    public double turretAngleRad;
    public double hoodAngleRad;
    public double flywheelVoltage;
    public double flywheelRPS;

    private Supplier<Double> turretAngleSupplierRad;
    private Supplier<Double> hoodDeltaSupplierRad;
    private Supplier<Double> flywheelDeltaSupplierRPM;

    private Supplier<double[]> velocitySupplier;

    private Supplier<double[]> desiredLinearVelocitySupplier;

    private Supplier<Pose2d> robotPoseSupplier;
    private Supplier<Pose2d> turretPoseSupplier;

    private Supplier<Double> flywheelRPMSupplier;

    private Supplier<Boolean> outgoingFuelSensorSupplier;

    private Translation2d targetPositionFudgeFactor = Translation2d.kZero;

    public ShootingSubStateManager() {
        super(new StateRequest<>(ShootingStateRequest.STANDBY, StateRequestPriority.NORMAL));
        // Java is evil so the arrays need to be prepopulated
        Arrays.fill(velocityFrames, new double[] {0,0,0});
        Arrays.fill(accelerationFrames, new double[] {0,0,0});
        Arrays.fill(jerkFrames, new double[] {0,0,0});
        Arrays.fill(timeFrames, System.currentTimeMillis());
    }

    @Override
    public void setupSuppliers() {
        turretAngleSupplierRad = TurretSubStateManager.getInstance()::getAngleRad;
        hoodDeltaSupplierRad = HoodSubStateManager.getInstance()::getDeltaRad;
        flywheelRPMSupplier = FlywheelSubStateManager.getInstance()::getRPM;
        flywheelDeltaSupplierRPM = () -> {return flywheelRPS*60*FlywheelConstants.gearRatio - flywheelRPMSupplier.get();};

        velocitySupplier = DriveSubStateManager.getInstance()::getRobotVelocitySOTM;
        desiredLinearVelocitySupplier = DriveSubStateManager.getInstance()::getDesiredLinearVelocitySOTM;
        robotPoseSupplier = DriveSubStateManager.getInstance()::getRobotPose;
        turretPoseSupplier = () -> {return robotPoseSupplier.get().transformBy(ShootingConstants.robotToTurret);};

        outgoingFuelSensorSupplier = TransitSubStateManager.getInstance()::getOutgoingFuelSensorTripped;
    }

    public boolean getIsCloseToTrench() {return closeToTrench;}

    public void targetPositionShiftLeft() {
        targetPositionFudgeFactor = targetPositionFudgeFactor.plus(new Translation2d(0, -ShootingConstants.positionFudgeFactorIncrement));
        LoggingUtils.logToAdvantageKit(targetPositionFudgeFactor, "FeedForward/targetPosFudge");
        System.out.println("Target Pos Fudge Value: " + targetPositionFudgeFactor);


    }
    public void targetPositionShiftRight() {
        targetPositionFudgeFactor = targetPositionFudgeFactor.plus(new Translation2d(0, ShootingConstants.positionFudgeFactorIncrement));
        LoggingUtils.logToAdvantageKit(targetPositionFudgeFactor, "FeedForward/targetPosFudge");
        System.out.println("Target Pos Fudge Value: " + targetPositionFudgeFactor);
    }
    public void targetPositionShiftForward() {
        targetPositionFudgeFactor = targetPositionFudgeFactor.plus(new Translation2d(ShootingConstants.positionFudgeFactorIncrement, 0));
        LoggingUtils.logToAdvantageKit(targetPositionFudgeFactor, "FeedForward/targetPosFudge");
        System.out.println("Target Pos Fudge Value: " + targetPositionFudgeFactor);
    }
    public void targetPositionShiftBackward() {
        targetPositionFudgeFactor = targetPositionFudgeFactor.plus(new Translation2d(-ShootingConstants.positionFudgeFactorIncrement, 0));
        LoggingUtils.logToAdvantageKit(targetPositionFudgeFactor, "FeedForward/targetPosFudge");
        System.out.println("Target Pos Fudge Value: " + targetPositionFudgeFactor);

    }
    public void targetPositionReset() {
        targetPositionFudgeFactor = Translation2d.kZero;
        LoggingUtils.logToAdvantageKit(targetPositionFudgeFactor, "FeedForward/targetPosFudge");
        System.out.println("Target Pos Fudge Value: " + targetPositionFudgeFactor);
    }

    public boolean getCanShoot() {
        if ((!robotOnCorrectSide && !robotInDepotThird && !robotInHumanPlayerThird) || closeToTrench) {
            Logger.recordOutput("Shooting/CanShoot", "BAD POSITION: " + robotOnCorrectSide + " " + robotInDepotThird + " " + robotInHumanPlayerThird + " " + closeToTrench);
            Keybinds.driverController.vibrate(0);
        } else if (!velocityInLargeBounds) {
            Logger.recordOutput("Shooting/CanShoot", "VERY BAD VELOCITY");
            Keybinds.driverController.vibrate(1);
        } else if (!velocityInBounds) {
            Logger.recordOutput("Shooting/CanShoot", "BAD VELOCITY");
            Keybinds.driverController.vibrate(.5);
        } else if (!accelerationInBounds) {
            Logger.recordOutput("Shooting/CanShoot", "BAD ACCELERATION");
            Keybinds.driverController.vibrate(.5);
        } else if (!jerkInBounds) {
            Logger.recordOutput("Shooting/CanShoot", "BAD JERK");
            Keybinds.driverController.vibrate(0);
        /* } else if (!distanceInLargeBounds) {
            Logger.recordOutput("Shooting/CanShoot", "VERY BAD DISTANCE");
            Keybinds.driverController.vibrate(0);
        } else if (!distanceInBounds) {
            Logger.recordOutput("Shooting/CanShoot", "BAD DISTANCE");
            Keybinds.driverController.vibrate(0);*/
        } else if (Math.abs(turretAngleSupplierRad.get() - turretAngleRad) > ShootingConstants.maxTurretDeltaRad) {
            Logger.recordOutput("Shooting/CanShoot", "BAD TURRET ANGLE: " + (turretAngleSupplierRad.get() - turretAngleRad));
            Keybinds.driverController.vibrate(0);
        } else if (Math.abs(hoodDeltaSupplierRad.get()) > ShootingConstants.maxHoodDeltaRad) {
            Logger.recordOutput("Shooting/CanShoot", "BAD HOOD ANGLE: " + hoodDeltaSupplierRad.get());
            Keybinds.driverController.vibrate(0);
        } else if (Math.abs(flywheelDeltaSupplierRPM.get()) > ShootingConstants.maxFlywheelDelta) {
            Logger.recordOutput("Shooting/CanShoot", "BAD FLYWHEEL DELTA: " + flywheelDeltaSupplierRPM.get());
            Keybinds.driverController.vibrate(0);
        } else if (!hubActiveWhenShotLands) {
            Logger.recordOutput("Shooting/CanShoot", "HUB INACTIVE");
            Keybinds.driverController.vibrate(0);
        } else {
            Logger.recordOutput("Shooting/CanShoot", "GOOD");
            CANdleSubStateManager.setLEDs(CANdleStateRequest.SHOOT_GOOD);
            Keybinds.driverController.vibrate(0);
            return true;
        }
        return false;
    }

    private PIDController flywheelFFController = new PIDController(0.00025, 0, 0);
    private RuntimeTunablePIDValues pidValues = new RuntimeTunablePIDValues("Tuning/FlywheelFF/PID", 0.00025, 0, 0, 0);


    public void periodic() {
        canShoot = getCanShoot();
        double[] shootingParameters = calculate();
        //Rotation2d testAngle = Keybinds.getTestAngle();
        //if (testAngle != null) turretAngleRad = testAngle.getRadians();//shootingParameters[0];
        turretAngleRad = shootingParameters[0];
        hoodAngleRad = shootingParameters[1];
        flywheelVoltage = FlywheelConstants.rpsToVoltage.get(shootingParameters[2]);
        flywheelRPS = shootingParameters[2];

        if (!MetaConstants.inProduction) {
            double[] newPidValues = pidValues.getUpdatedPIDConstants();
            flywheelFFController.setPID(newPidValues[0], newPidValues[1], newPidValues[2]);
        }

        flywheelVoltage += flywheelFFController.calculate(flywheelRPMSupplier.get(), flywheelRPS*60*FlywheelConstants.gearRatio);

        Logger.recordOutput("Shooting/DesiredTurretAngleRad", turretAngleRad);
        Logger.recordOutput("Shooting/DesiredHoodAngleRad", hoodAngleRad);
        Logger.recordOutput("Shooting/DesiredFlywheelSpeedRPS", flywheelRPS);

        switch (activeStateRequest.getStateRequestType()) {
            case SHOOT_PRELOADS:
                if (!outgoingFuelSensorSupplier.get()) { // All preloads must have been shot
                    this.activeStateRequest.updateStatus(StateRequestStatus.FULFILLED);
                }
            case SHOOT:
                this.activeStateRequest.updateStatus(StateRequestStatus.FULFILLED);
                break;
            case FORCE_SHOOT:
                this.activeStateRequest.updateStatus(StateRequestStatus.FULFILLED);
                break;
            default: break;
        }
    }

    public boolean getShooting() {
        switch (activeStateRequest.getStateRequestType()) {
            case SHOOT: return canShoot;
            case SHOOT_PRELOADS: return this.activeStateRequest.getStatus() != StateRequestStatus.FULFILLED; // Only shoot if we haven't shot all preloads (we will verify if shot is possible in testing)
            case FORCE_SHOOT: return true;
            default: return false;
        }
    }

    public boolean getShootingIntent() {
        ShootingStateRequest stateRequestType = activeStateRequest.getStateRequestType();
        return stateRequestType == ShootingStateRequest.SHOOT || stateRequestType == ShootingStateRequest.FORCE_SHOOT || (stateRequestType == ShootingStateRequest.SHOOT_PRELOADS && activeStateRequest.getStatus() != StateRequestStatus.FULFILLED);
    }

    public double getTurretAngleRad() {
        return turretAngleRad;
    }

    RuntimeTunableValue hoodAngle = new RuntimeTunableValue("Tuning/Shooting/Angle", 0.0);
    RuntimeTunableValue rpm = new RuntimeTunableValue("Tuning/Shooting/RPM", 0.0);


    public double getHoodAngleRad() {
        return hoodAngleRad;
        //return (double) hoodAngle.getValue();
    }

    public double getFlywheelVoltage() {
        //return FlywheelConstants.rpsToVoltage.get((double) rpm.getValue());
        return flywheelVoltage;
    }

    
    // Calculation runtime variables
    public Translation2d targetPosition = new Translation2d(4.625594, 4.034536);
    public Translation2d depotBackPosition = new Translation2d(2.784, 6.357);
    public Translation2d humanPlayerBackPosition = new Translation2d(2.784, 1.894);

    private final int framesOfVelocityMeasurement = 6;

    private double[][] velocityFrames =  new double[framesOfVelocityMeasurement][3];
    private double[][] accelerationFrames =  new double[framesOfVelocityMeasurement][3];
    private double[][] jerkFrames =  new double[framesOfVelocityMeasurement][3];
    private long[] timeFrames = new long[framesOfVelocityMeasurement];

    private double lookaheadTimeSeconds = 0.005;

    private double nextShotTime = -1;
    private boolean hasShotInCurrentPhase;

    private boolean velocityInBounds = true;
    private boolean velocityInLargeBounds = true;
    private boolean accelerationInBounds = true;
    private boolean jerkInBounds = true;
    private boolean robotOnCorrectSide = true;
    private boolean robotInDepotThird = false;
    private boolean robotInHumanPlayerThird = false;

    private boolean distanceInBounds = true;
    private boolean distanceInLargeBounds = true;

    private boolean hubActiveWhenShotLands = true;

    // Helpers

    public double getAngle(double distance) {
        return ShootingConstants.baseTable.get(distance, 1, 0);
    }
    public double getTimeOfFlight(double distance) {
        return ShootingConstants.baseTable.get(distance, 3, 0);
    }
    public double getRPM(double distance) {
        return ShootingConstants.baseTable.get(distance, 2, 0);
    }

    public boolean getDoAim() {
        return robotOnCorrectSide || ((robotInDepotThird || robotInHumanPlayerThird) && getShootingIntent());
    }

    public RuntimeTunableValue dragCoeff = new RuntimeTunableValue("/SOTM/DragCoefficient", .1);
    public RuntimeTunableValue lookaheadTimeSec = new RuntimeTunableValue("/SOTM/LookaheadTimeSec", .005);

    /**
     * Calculates optimal values for shooter control systems (shoot-on-the-move)
     * @return array containing turret angle in radians, hood angle in radians, and flywheel voltage
     */
    public double[] calculate()
    {
        double[] velocity = velocitySupplier.get(); 

        Pose2d turretPose = turretPoseSupplier.get();


        double turretVelocityX = velocity[0] + velocity[2]
            * (ShootingConstants.robotToTurret.getY() * turretPose.getRotation().getCos()
                * ShootingConstants.robotToTurret.getX() * turretPose.getRotation().getSin());
        
        double turretVelocityY = velocity[1] + velocity[2]
            * (ShootingConstants.robotToTurret.getX() * turretPose.getRotation().getCos()
                * ShootingConstants.robotToTurret.getY() * turretPose.getRotation().getSin());

        velocity = new double[] {turretVelocityX, turretVelocityY, velocity[2]};

        double velocityNorm = Math.sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1]);


        
        for (int frame = framesOfVelocityMeasurement-2; frame >= 0; frame--)
        {
            velocityFrames[frame + 1] = velocityFrames[frame];
            accelerationFrames[frame + 1] = accelerationFrames[frame];
            jerkFrames[frame + 1] = jerkFrames[frame];
            timeFrames[frame + 1] = timeFrames[frame];
        }

        timeFrames[0] = System.currentTimeMillis();
        velocityFrames[0] = new double[] {velocity[0], velocity[1], velocity[2]};

        double deltaTimeSec = (timeFrames[0] - timeFrames[framesOfVelocityMeasurement-1]) / 1000.0;

        accelerationFrames[0] = new double[] {
            (velocityFrames[0][0]-velocityFrames[framesOfVelocityMeasurement-1][0])/deltaTimeSec,
            (velocityFrames[0][1]-velocityFrames[framesOfVelocityMeasurement-1][1])/deltaTimeSec,
            (velocityFrames[0][2]-velocityFrames[framesOfVelocityMeasurement-1][2])/deltaTimeSec,
        };

        jerkFrames[0] = new double[] {
            (accelerationFrames[0][0]-accelerationFrames[framesOfVelocityMeasurement-1][0])/deltaTimeSec,
            (accelerationFrames[0][1]-accelerationFrames[framesOfVelocityMeasurement-1][1])/deltaTimeSec,
            (accelerationFrames[0][2]-accelerationFrames[framesOfVelocityMeasurement-1][2])/deltaTimeSec,
        };
        
        velocityInBounds = velocityNorm < ShootingConstants.maxVelocity;
        accelerationInBounds = Math.sqrt(
            accelerationFrames[0][0] * accelerationFrames[0][0] + accelerationFrames[0][1] * accelerationFrames[0][1]) 
            < ShootingConstants.maxAcceleration;
        jerkInBounds = true;//jerkFrames[0].getNorm() < ShootingConstants.maxJerk;
        velocityInLargeBounds = velocityNorm < ShootingConstants.maxVelocityLarge;

        double[] extraVelocity = new double[] {0, 0, 0};

        double currentTime = System.currentTimeMillis() / 1000.0;

        double lookaheadTime = nextShotTime - currentTime;

        
        extraVelocity = new double[] {
            extraVelocity[0] + accelerationFrames[0][0] * lookaheadTime, 
            extraVelocity[1] + accelerationFrames[0][1] * lookaheadTime, 
            extraVelocity[2] + accelerationFrames[0][2] * lookaheadTime};
        //extraVelocity.plus(accelerationFrames[0].times(lookaheadTime));//.plus(jerkFrames[0].times(0.5 * lookaheadTime * lookaheadTime));

        double[] predictedVelocity = velocity;
        if (!Double.isNaN(extraVelocity[0]) && !Double.isNaN(extraVelocity[1])) predictedVelocity = new double[] {
            velocity[0] + extraVelocity[0],
            velocity[1] + extraVelocity[1],
            velocity[2] + extraVelocity[2]
        };
        Logger.recordOutput("/Turret/Control/ExtraVelocity", extraVelocity);
        Logger.recordOutput("/Turret/Control/Velocity", velocityFrames[0]);
        Logger.recordOutput("/Turret/Control/Acceleration", accelerationFrames[0]);
        Logger.recordOutput("/Turret/Control/Jerk", jerkFrames[0]);

        double[] desiredLinearVelocity = desiredLinearVelocitySupplier.get();
        if (desiredLinearVelocity == null) desiredLinearVelocity = velocity;

        velocity = new double[] {
            velocity[0]/2 + /*predictedVelocity[0]/3 + */desiredLinearVelocity[0]/2,
            velocity[1]/2 + /*predictedVelocity[1]/3 + */desiredLinearVelocity[1]/2,
            velocity[2]/2 + /*predictedVelocity[2]/3 + */desiredLinearVelocity[2]/2

        };

        nextShotTime = currentTime + (double)lookaheadTimeSec.getValue();//lookaheadTimeSeconds;

        turretPose = new Pose2d(turretPose.getX(), turretPose.getY(), turretPose.getRotation().times(-1));
        robotOnCorrectSide = turretPose.getX() < ShootingConstants.maxTurretX;
        robotInDepotThird = turretPose.getY() > 4.75;
        robotInHumanPlayerThird = turretPose.getY() < 3.5;
        Translation2d turretPoseTranslation = turretPose.getTranslation();
        closeToTrench = 
            ShootingConstants.trenchBoundsHumanPlayerOwnSide.contains(turretPoseTranslation) ||
            ShootingConstants.trenchBoundsDepotOwnSide.contains(turretPoseTranslation) ||
            ShootingConstants.trenchBoundsHumanPlayerOppositeSide.contains(turretPoseTranslation) ||
            ShootingConstants.trenchBoundsDepotOppositeSide.contains(turretPoseTranslation);

        Translation2d lookaheadDelta = new Translation2d(
            velocity[0] * (nextShotTime-currentTime),
            velocity[1] * (nextShotTime-currentTime)
        );
        turretPose = new Pose2d(turretPoseTranslation.plus(lookaheadDelta),turretPose.getRotation());

        Translation2d targetPosition = robotOnCorrectSide ? this.targetPosition.plus(targetPositionFudgeFactor) : 
        (robotInDepotThird ? this.depotBackPosition : this.humanPlayerBackPosition);
        Logger.recordOutput("Tuning/TargetPosition", targetPosition);

        Translation2d targetDifference = targetPosition.minus(turretPoseTranslation);
        
        double offsetDistance = targetDifference.getNorm();
        Logger.recordOutput("Tuning/Distance", offsetDistance);

        distanceInBounds = (offsetDistance < ShootingConstants.maxDistanceFromGoal);
        distanceInLargeBounds = (offsetDistance < ShootingConstants.maxDistanceFromGoalLarge);

        

        Translation2d distanceVector = new Translation2d();
        Rotation2d turretAngle = Rotation2d.kZero;
        double timeOfFlight = getTimeOfFlight(offsetDistance);
        double hoodAngle = getAngle(offsetDistance);
        double flywheelRPM = getRPM(offsetDistance);
        Translation2d requiredTotalVelocity = targetDifference.div(timeOfFlight);

        for (int i = 0; i < ShootingConstants.numberOfAlgorithmIterations; i++)
        {

            Translation2d requiredImpartedVelocity = requiredTotalVelocity.minus(new Translation2d(velocity[0], velocity[1]));


            distanceVector = requiredImpartedVelocity.times(timeOfFlight);

            double groundDistance = distanceVector.getNorm();

            // 1. Account for Drag: "Virtual Distance"
            // The ball loses energy over time. We pretend the target is further away.
            // A simple approximation: Dist_virtual = Dist_actual * (1 + k * Dist_actual)
            double virtualDistance = groundDistance * (1 + ShootingConstants.dragCoefficient * groundDistance);

            // 2. Pass the VIRTUAL distance to your LUT
            hoodAngle = getAngle(virtualDistance);
            flywheelRPM = getRPM(virtualDistance);
            timeOfFlight = getTimeOfFlight(virtualDistance);

            // 3. Recalculate based on the new timeOfFlight from the LUT
            requiredTotalVelocity = targetDifference.div(timeOfFlight);
            requiredImpartedVelocity = requiredTotalVelocity.minus(new Translation2d(velocity[0], velocity[1]));
            
            // ... update turretAngle ...

            // 4) Rotate into ROBOT frame (turret frame if turret is robot-relative)
            
            double cosYaw = turretPose.getRotation().getCos();
            double sinYaw = turretPose.getRotation().getSin();

            Translation2d V_turret = new Translation2d(
                requiredImpartedVelocity.getX() * cosYaw - requiredImpartedVelocity.getY() * sinYaw,
                requiredImpartedVelocity.getX() * sinYaw + requiredImpartedVelocity.getY() * cosYaw
            );

            // 5) Turret angle command
            turretAngle = V_turret.getAngle().times(-1);

        }
        if (Robot.timeUntilHubIsActive >= 0) {
            hubActiveWhenShotLands = (timeOfFlight >= Robot.timeUntilHubIsActive);
        } else {
            // Don't evaluate if we'll make it in time
            hubActiveWhenShotLands = true;//(timeOfFlight < Math.abs(Robot.timeUntilHubIsActive));
        }
        
        return new double[] {turretAngle.getRadians(), hoodAngle, flywheelRPM};
        
    }

    public static ShootingSubStateManager getInstance() {
        return (ShootingSubStateManager) MainStateManager.getInstance().resolveSubStateManager(ShootingStateRequest.class);
    }

    public String toString() {
        return "Shooting SubStateManager";
    }
}
