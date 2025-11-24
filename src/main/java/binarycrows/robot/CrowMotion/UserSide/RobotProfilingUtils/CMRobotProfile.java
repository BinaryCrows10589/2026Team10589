package binarycrows.robot.CrowMotion.UserSide.RobotProfilingUtils;

import java.util.function.Supplier;

/**
 * Represents a physical profile for the robot, defining its motion capabilities and dynamics. 
 * Used for the trajectory symulation
 */
public class CMRobotProfile {

    /** The maximum achievable translational velocity in meters per second (m/s). */
    private final double maxPossibleTranslationalVelocityMPS;

    /** The maximum achievable rotational velocity in degrees per second (deg/s). */
    private final double maxPossibleRotationalVelocityDPS;

    /** The maximum speed of all swerve modules averaged in meters per second (m/s). */
    private final double maxPossibleAverageSwerveModuleMPS;
    
    /**
     * A function mapping velocity to acceleration. 
     */
    private final Supplier<Double> velocityVSAccelerationFunction;
    
    /**
     * WARNING!!! These values MUST be accurrete to the true physical limits of the robot.
     * Use the profiling methods in the RobotProfiling utill file.
     * Instructions for how to complete each of the profiles are described within the RobotProfilingUtil
     * Constructs a new RobotProfile with the specified physical parameters.
     *
     * @param maxPossibleTranslationalVelocityMPS Maximum translational speed in m/s
     * @param maxPossibleRotationalVelocityDPS Maximum rotational speed in degrees per second
     * @param maxPossibleAverageSwerveModuleMPS Max average swerve module velocity in m/s
     * @param velocityVSAccelerationFunction Function that returns acceleration given current velocity
     */
    public CMRobotProfile(
        double maxPossibleTranslationalVelocityMPS,
        double maxPossibleRotationalVelocityDPS,
        double maxPossibleAverageSwerveModuleMPS,
        Supplier<Double> velocityVSAccelerationFunction
    ) {
        this.maxPossibleTranslationalVelocityMPS = maxPossibleTranslationalVelocityMPS;
        this.maxPossibleRotationalVelocityDPS = maxPossibleRotationalVelocityDPS;
        this.maxPossibleAverageSwerveModuleMPS = maxPossibleAverageSwerveModuleMPS;
        this.velocityVSAccelerationFunction = velocityVSAccelerationFunction;
    }

    /**
     * @return The robot's maximum translational velocity in m/s.
     */
    public double getMaxPossibleTranslationalVelocityMPS() {
        return this.maxPossibleTranslationalVelocityMPS;
    }

    /**
     * @return The robot's maximum rotational velocity in degrees per second.
     */
    public double getMaxPossibleRotationalVelocityDPS() {
        return this.maxPossibleRotationalVelocityDPS;
    }

    /**
     * @return The robot's maximum average swerve module speed in m/s.
     */
    public double getMaxPossibleAverageSwerveModuleMPS() {
        return this.maxPossibleAverageSwerveModuleMPS;
    }

    /**
     * @return The current acceleration value based on the velocity-to-acceleration function.
     */
    public double getVelocityVSAccelerationFunction() {
        return this.velocityVSAccelerationFunction.get();
    }

}
