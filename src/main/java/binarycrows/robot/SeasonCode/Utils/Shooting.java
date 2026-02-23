package binarycrows.robot.SeasonCode.Utils;

import binarycrows.robot.SeasonCode.SubStateManagers.Flywheel.FlywheelSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Hood.HoodSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.SwerveDrive.DriveSubStateManager;
import binarycrows.robot.SeasonCode.SubStateManagers.Turret.TurretSubStateManager;
import binarycrows.robot.Utils.ConversionUtils;
import binarycrows.robot.Utils.LerpTable;
import binarycrows.robot.Utils.UnkeyedLerpTable;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Time;

public class Shooting { //TODO: WIP
    public static boolean isShooting = false;
    public static boolean isForceShooting = false;
    public static boolean canShoot = false;

    public static double turretAngleRad;
    public static double hoodAngleRad;
    public static double flywheelVoltage;

    public static boolean getCanShoot() { //TODO: Implement this and make it more better
        return chassisSpeedsInBounds && 
        TurretSubStateManager.getInstance().getDelta() < maxTurretDelta &&
        HoodSubStateManager.getInstance().getDelta() < maxHoodDelta &&
        FlywheelSubStateManager.getInstance().getDelta() < maxFlywheelDelta;
    }

    public static void periodic() {
        canShoot = getCanShoot();
        double[] shootingParameters = calculate();
        turretAngleRad = shootingParameters[0];
        hoodAngleRad = shootingParameters[1];
        flywheelVoltage = shootingParameters[2];
    }

    public static boolean getShooting() {
        return isShooting && canShoot || isForceShooting;
    }


    // Base table goes distance, hood angle, flywheel voltage, time of flight
    public static final UnkeyedLerpTable baseTable = new UnkeyedLerpTable(new double[][] {
        //           |distance|hood angle|flywheel|ToF|   
        new double[] {0,       0,         0,       0}, // TODO: after tuning, do NOT leave 0,0,0 as a value
    }, 
    false);

    // Constants
    public static final Translation2d targetPosition = new Translation2d(ConversionUtils.inchesToMeters(182.11), ConversionUtils.inchesToMeters(158.84));
    public static final Transform2d robotToTurret = new Transform2d(new Translation2d(0, 0), Rotation2d.kZero); //TODO: Set to correct translation
    public static final int numberOfAlgorithmIterations = 20; //TODO: This can certainly be much lower...
    public static final double dragCoefficient = 0.2; //TODO: tune... ...a lot

    public static final double maxTurretDelta = 10;
    public static final double maxHoodDelta = 10;
    public static final double maxFlywheelDelta = 10;
    public static final double maxVelocity = 4.4;
    public static final double maxAcceleration = 4.4;
    public static final double maxJerk = 4.4;

    // Calculation runtime variables
    private static Translation2d[][] derivativesOfVelocity =  new Translation2d[3][2];
    private static Translation2d[] valuesOfDerivatiesOfVelocity = new Translation2d[2];

    private static double lookaheadTimeSeconds = 0.2f;
    private static double phaseTimeSeconds = 0.03f;

    private static double nextShotTime = -1;
    private static boolean hasShotInCurrentPhase;

    private static boolean chassisSpeedsInBounds = true;

    // Helpers
    public static Translation2d getLinearVelocity() {
        return DriveSubStateManager.getInstance().getLinearVelocitySOTM();
    }

    public static Pose2d getTurretPose() {
        return DriveSubStateManager.getInstance().getRobotPose().transformBy(robotToTurret);
    }

    public static double getTimeOfFlight(double distance) {
        return baseTable.get(distance, 3, 0);
    }
    public static double getAngle(double distance) {
        return baseTable.get(distance, 1, 0);
    }
    public static double getVoltage(double distance) {
        return baseTable.get(distance, 2, 0);
    }

    /**
     * Calculates optimal values for shooter control systems (shoot-on-the-move)
     * @return array containing turret angle in radians, hood angle in radians, and flywheel voltage
     */
    public static double[] calculate()
    {
        Translation2d velocity = getLinearVelocity(); 

        int numFrames = derivativesOfVelocity[0].length;
        int numDerivatives = derivativesOfVelocity.length-1;
        for (int derivative = 0; derivative < numDerivatives; derivative++) {
            
            for (int frame = 0; frame < numFrames-1; frame++)
            {
                derivativesOfVelocity[derivative][frame] = derivativesOfVelocity[derivative][frame + 1];
            }
        }

        derivativesOfVelocity[0][numFrames-1] = velocity;

        for (int derivative = 0; derivative < numDerivatives; derivative++)
        {   
            valuesOfDerivatiesOfVelocity[derivative] = 
            (derivativesOfVelocity[derivative][numFrames-1].minus(derivativesOfVelocity[derivative][0]))
            .div(0.02)
            .div(numFrames);
            derivativesOfVelocity[derivative+1][numFrames-1] = valuesOfDerivatiesOfVelocity[derivative];
        }

        chassisSpeedsInBounds = velocity.getNorm() < maxVelocity && valuesOfDerivatiesOfVelocity[0].getNorm() < maxAcceleration && valuesOfDerivatiesOfVelocity[1].getNorm() < maxJerk;

        Translation2d extraVelocity = Translation2d.kZero;

        double currentTime = System.currentTimeMillis() * 1000;

        for (int derivative = numDerivatives - 1; derivative >= 0; derivative--)
        {
            extraVelocity = extraVelocity.plus(valuesOfDerivatiesOfVelocity[derivative].times(nextShotTime - currentTime));
        }

        velocity = velocity.plus(extraVelocity);

        if (nextShotTime < currentTime)
        {
            hasShotInCurrentPhase = false;
            nextShotTime = currentTime + lookaheadTimeSeconds + phaseTimeSeconds;
        } else if (nextShotTime < currentTime + phaseTimeSeconds && !hasShotInCurrentPhase)
        {
            // TODO: The phase time will match the shooting rate we already have?
           // if (Math.Abs(hoodController.turretDelta) < 8 && Math.Abs(hoodController.hoodDelta) < 6) shooterController.shootNoInput();
            hasShotInCurrentPhase = true;
        }
        Pose2d turretPose = getTurretPose();

        Translation2d lookaheadDelta = velocity.times(nextShotTime-currentTime);
        turretPose = new Pose2d(turretPose.getTranslation().plus(lookaheadDelta),turretPose.getRotation());

        Translation2d targetDifference = targetPosition.minus(turretPose.getTranslation());
        
        double offsetDistance = targetDifference.getNorm();
        Translation2d distanceVector = new Translation2d();
        Rotation2d turretAngle = Rotation2d.kZero;
        double timeOfFlight = getTimeOfFlight(offsetDistance);
        double hoodAngle = getAngle(offsetDistance);
        double flywheelVoltage = getVoltage(offsetDistance);
        Translation2d requiredTotalVelocity = targetDifference.div(timeOfFlight);

        for (int i = 0; i < numberOfAlgorithmIterations; i++)
        {

            Translation2d requiredImpartedVelocity = requiredTotalVelocity.minus(velocity);


            distanceVector = requiredImpartedVelocity.times(timeOfFlight);

            double groundDistance = distanceVector.getNorm();

            // 1. Account for Drag: "Virtual Distance"
            // The ball loses energy over time. We pretend the target is further away.
            // A simple approximation: Dist_virtual = Dist_actual * (1 + k * Dist_actual)
            double virtualDistance = groundDistance * (1 + dragCoefficient * groundDistance);

            // 2. Pass the VIRTUAL distance to your LUT
            hoodAngle = getAngle(virtualDistance);
            flywheelVoltage = getVoltage(virtualDistance);
            timeOfFlight = getTimeOfFlight(virtualDistance);

            // 3. Recalculate based on the new timeOfFlight from the LUT
            requiredTotalVelocity = targetDifference.div(timeOfFlight);
            requiredImpartedVelocity = requiredTotalVelocity.minus(velocity);
            
            // ... update turretAngle ...

            // 4) Rotate into ROBOT frame (turret frame if turret is robot-relative)
            
            double cosYaw = turretPose.getRotation().getCos(); //TODO: For some reason these were negative in Unity
            double sinYaw = turretPose.getRotation().getSin();

            Translation2d V_turret = new Translation2d(
                requiredImpartedVelocity.getX() * cosYaw - requiredImpartedVelocity.getY() * sinYaw,
                requiredImpartedVelocity.getX() * sinYaw + requiredImpartedVelocity.getY() * cosYaw
            );

            // 5) Turret angle command
            turretAngle = V_turret.getAngle();

        }
        
        return new double[] {turretAngle.getRadians(), hoodAngle, flywheelVoltage};
    }
}
