package binarycrows.robot.SeasonCode.SubStateManagers.Shooting;

import java.util.Arrays;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import binarycrows.robot.MainStateManager;
import binarycrows.robot.Robot;
import binarycrows.robot.StateRequest;
import binarycrows.robot.SubStateManager;
import binarycrows.robot.Enums.StateRequestPriority;
import binarycrows.robot.Enums.StateRequestStatus;
import binarycrows.robot.SeasonCode.Constants.FlywheelConstants;
import binarycrows.robot.SeasonCode.Constants.ShootingConstants;
import binarycrows.robot.SeasonCode.SubStateManagers.CANdle.CANdleStateRequest;
import binarycrows.robot.SeasonCode.SubStateManagers.CANdle.CANdleSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Flywheel.FlywheelSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Hood.HoodSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Transit.TransitSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Turret.TurretSubStateManager;
import binarycrows.robot.Utils.Tuning.RuntimeTunableValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ShootingSubStateManager extends SubStateManager<ShootingStateRequest> {
    private boolean canShoot = false;

    private boolean closeToTrench = false;

    public double turretAngleRad;
    public double hoodAngleRad;
    public double flywheelVoltage;
    public double flywheelRPM;

    private Supplier<Double> turretDeltaSupplierRad;
    private Supplier<Double> hoodDeltaSupplierRad;
    private Supplier<Double> flywheelDeltaSupplierRPM;

    private Supplier<Translation2d> linearVelocitySupplier;
    private Supplier<Pose2d> robotPoseSupplier;
    private Supplier<Pose2d> turretPoseSupplier;

    private Supplier<Double> flywheelRPMSupplier;

    private Supplier<Boolean> outgoingFuelSensorSupplier;

    public ShootingSubStateManager() {
        super(new StateRequest<>(ShootingStateRequest.STANDBY, StateRequestPriority.NORMAL));
        // Java is evil so the arrays need to be prepopulated
        Arrays.fill(velocityFrames, Translation2d.kZero);
        Arrays.fill(accelerationFrames, Translation2d.kZero);
        Arrays.fill(jerkFrames, Translation2d.kZero);
        Arrays.fill(timeFrames, System.currentTimeMillis());
        
    }

    @Override
    public void setupSuppliers() {
        turretDeltaSupplierRad = TurretSubStateManager.getInstance()::getDeltaRad;
        hoodDeltaSupplierRad = HoodSubStateManager.getInstance()::getDeltaRad;
        flywheelRPMSupplier = FlywheelSubStateManager.getInstance()::getRPM;
        flywheelDeltaSupplierRPM = () -> {return flywheelRPM - flywheelRPMSupplier.get();};

        linearVelocitySupplier = DriveSubStateManager.getInstance()::getLinearVelocitySOTM;
        robotPoseSupplier = DriveSubStateManager.getInstance()::getRobotPose;
        turretPoseSupplier = () -> {return robotPoseSupplier.get().transformBy(ShootingConstants.robotToTurret);};

        outgoingFuelSensorSupplier = TransitSubStateManager.getInstance()::getOutgoingFuelSensorTripped;
    }

    public boolean getIsCloseToTrench() {return closeToTrench;}


    // TODO: Put reason on dashboard, maybe make Isaac's controller vibrate when we know a mechanism is screwed up (CAN stale, not moving, outside of mechanical range)
    public boolean getCanShoot() { // TODO: LED Suggestion is to have color for: has balls, full; can't shoot shown by flashing; 
        if ((!robotOnCorrectSide && !robotInDepotThird && !robotInHumanPlayerThird) || closeToTrench) {
            CANdleSubStateManager.setLEDs(CANdleStateRequest.SHOOT_BAD_WRONG_SIDE_OF_FIELD);
            Logger.recordOutput("Shooting/CanShoot", "BAD POSITION: " + robotOnCorrectSide + " " + robotInDepotThird + " " + robotInHumanPlayerThird + " " + closeToTrench);
        } else if (!velocityInLargeBounds) {
            Logger.recordOutput("Shooting/CanShoot", "VERY BAD VELOCITY");
            CANdleSubStateManager.setLEDs(CANdleStateRequest.SHOOT_BAD_VELOCITY_WAY_TOO_HIGH);
        } else if (!velocityInBounds) {
            Logger.recordOutput("Shooting/CanShoot", "BAD VELOCITY");
            CANdleSubStateManager.setLEDs(CANdleStateRequest.SHOOT_BAD_VELOCITY_TOO_HIGH);
        } else if (!accelerationInBounds) {
            Logger.recordOutput("Shooting/CanShoot", "BAD ACCELERATION");
            CANdleSubStateManager.setLEDs(CANdleStateRequest.SHOOT_BAD_ACCELERATION_TOO_HIGH);
        } else if (!jerkInBounds) {
            Logger.recordOutput("Shooting/CanShoot", "BAD JERK");
            CANdleSubStateManager.setLEDs(CANdleStateRequest.SHOOT_BAD_JERK_TOO_HIGH);
        } else if (!distanceInLargeBounds) {
            Logger.recordOutput("Shooting/CanShoot", "VERY BAD DISTANCE");
            CANdleSubStateManager.setLEDs(CANdleStateRequest.SHOOT_BAD_DISTANCE_WAY_TOO_HIGH);
        } else if (!distanceInBounds) {
            Logger.recordOutput("Shooting/CanShoot", "BAD DISTANCE");
            CANdleSubStateManager.setLEDs(CANdleStateRequest.SHOOT_BAD_DISTANCE_TOO_HIGH);
        } else if (Math.abs(turretDeltaSupplierRad.get()) > ShootingConstants.maxTurretDeltaRad) {
            Logger.recordOutput("Shooting/CanShoot", "BAD TURRET ANGLE");
            CANdleSubStateManager.setLEDs(CANdleStateRequest.SHOOT_BAD_TURRET_DELTA_TOO_HIGH);
        } else if (Math.abs(hoodDeltaSupplierRad.get()) > ShootingConstants.maxHoodDeltaRad) {
            Logger.recordOutput("Shooting/CanShoot", "BAD HOOD ANGLE");
            CANdleSubStateManager.setLEDs(CANdleStateRequest.SHOOT_BAD_HOOD_DELTA_TOO_HIGH);
        } else if (Math.abs(flywheelDeltaSupplierRPM.get()) > ShootingConstants.maxFlywheelDelta) {
            Logger.recordOutput("Shooting/CanShoot", "BAD FLYWHEEL DELTA");
            CANdleSubStateManager.setLEDs(CANdleStateRequest.SHOOT_BAD_FLYWHEEL_DELTA_TOO_HIGH);
        //} else if (!hubActiveWhenShotLands) {

        } else {
            Logger.recordOutput("Shooting/CanShoot", "GOOD");
            CANdleSubStateManager.setLEDs(CANdleStateRequest.SHOOT_GOOD);
            return true;
        }
        return false;
    }

    public void periodic() {
        canShoot = getCanShoot();
        double[] shootingParameters = calculate();
        //Rotation2d testAngle = Keybinds.getTestAngle();
        //if (testAngle != null) turretAngleRad = testAngle.getRadians();//shootingParameters[0];
        turretAngleRad = shootingParameters[0];
        hoodAngleRad = shootingParameters[1];
        flywheelVoltage = FlywheelConstants.rpmToVoltage.get(shootingParameters[2]);
        flywheelRPM = shootingParameters[2];

        switch (activeStateRequest.getStateRequestType()) {
            case SHOOT_PRELOADS:
                if (!outgoingFuelSensorSupplier.get()) { // All preloads must have been shot
                    this.activeStateRequest.updateStatus(StateRequestStatus.FULFILLED);
                }
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
        return stateRequestType == ShootingStateRequest.SHOOT || stateRequestType == ShootingStateRequest.FORCE_SHOOT;
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
        //return FlywheelConstants.rpmToVoltage.get((double) rpm.getValue());
        return flywheelVoltage;
    }

    
    // Calculation runtime variables
    public Translation2d targetPosition = new Translation2d(4.625594, 4.034536);
    public Translation2d depotBackPosition = new Translation2d(2.784, 6.357);
    public Translation2d humanPlayerBackPosition = new Translation2d(2.784, 1.894);

    private final int framesOfVelocityMeasurement = 6;

    private Translation2d[] velocityFrames =  new Translation2d[framesOfVelocityMeasurement];
    private Translation2d[] accelerationFrames =  new Translation2d[framesOfVelocityMeasurement];
    private Translation2d[] jerkFrames =  new Translation2d[framesOfVelocityMeasurement];
    private long[] timeFrames = new long[framesOfVelocityMeasurement];

    private double lookaheadTimeSeconds = 0.2f;
    private double phaseTimeSeconds = 0.03f;

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

    /**
     * Calculates optimal values for shooter control systems (shoot-on-the-move)
     * @return array containing turret angle in radians, hood angle in radians, and flywheel voltage
     */
    public double[] calculate()
    {
        Translation2d velocity = linearVelocitySupplier.get(); 
        double velocityNorm = velocity.getNorm();

        
        for (int frame = framesOfVelocityMeasurement-2; frame >= 0; frame--)
        {
            velocityFrames[frame + 1] = velocityFrames[frame];
            accelerationFrames[frame + 1] = accelerationFrames[frame];
            jerkFrames[frame + 1] = jerkFrames[frame];
            timeFrames[frame + 1] = timeFrames[frame];
        }

        timeFrames[0] = System.currentTimeMillis();
        velocityFrames[0] = new Translation2d(velocity.getX(), velocity.getY());

        double deltaTimeSec = (timeFrames[0] - timeFrames[framesOfVelocityMeasurement-1]) / 1000.0;

        accelerationFrames[0] = velocityFrames[0].minus(velocityFrames[framesOfVelocityMeasurement-1]).div(deltaTimeSec);

        jerkFrames[0] = accelerationFrames[0].minus(accelerationFrames[framesOfVelocityMeasurement-1]).div(deltaTimeSec);

        velocityInBounds = velocityNorm < ShootingConstants.maxVelocity;
        accelerationInBounds = accelerationFrames[0].getNorm() < ShootingConstants.maxAcceleration;
        jerkInBounds = jerkFrames[0].getNorm() < ShootingConstants.maxJerk;
        velocityInLargeBounds = velocityNorm < ShootingConstants.maxVelocityLarge;

        Translation2d extraVelocity = Translation2d.kZero;

        double currentTime = System.currentTimeMillis() / 1000.0;

        double lookaheadTime = nextShotTime - currentTime;

        extraVelocity = extraVelocity.plus(accelerationFrames[0].times(lookaheadTime)).plus(jerkFrames[0].times(0.5 * lookaheadTime * lookaheadTime));

        if (!Double.isNaN(extraVelocity.getX()) && !Double.isNaN(extraVelocity.getY())) velocity = velocity.plus(extraVelocity);
        Logger.recordOutput("/Turret/Control/ExtraVelocity", extraVelocity);
        Logger.recordOutput("/Turret/Control/Velocity", velocityFrames[0]);
        Logger.recordOutput("/Turret/Control/Acceleration", accelerationFrames[0]);
        Logger.recordOutput("/Turret/Control/Jerk", jerkFrames[0]);

        if (nextShotTime < currentTime)
        {
            hasShotInCurrentPhase = false;
            nextShotTime = currentTime + lookaheadTimeSeconds + phaseTimeSeconds;
        } else if (nextShotTime < currentTime + phaseTimeSeconds && !hasShotInCurrentPhase)
        {
            hasShotInCurrentPhase = true;
        }
        Pose2d turretPose = turretPoseSupplier.get();
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

        Translation2d lookaheadDelta = velocity.times(nextShotTime-currentTime);
        turretPose = new Pose2d(turretPoseTranslation.plus(lookaheadDelta),turretPose.getRotation());

        Translation2d targetPosition = robotOnCorrectSide ? this.targetPosition : 
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

            Translation2d requiredImpartedVelocity = requiredTotalVelocity.minus(velocity);


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
            requiredImpartedVelocity = requiredTotalVelocity.minus(velocity);
            
            // ... update turretAngle ...

            // 4) Rotate into ROBOT frame (turret frame if turret is robot-relative)
            
            double cosYaw = turretPose.getRotation().getCos(); //TO-DO: For some reason these were negative in Unity
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
}
