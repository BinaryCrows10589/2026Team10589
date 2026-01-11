package binarycrows.robot.CrowMotion.UserSide;

public class CMRotation {
    private double angleDegrees;
    private int rotationDirection;
    private double completeRotationPercent;
    private double maxRotationVelocityDegrees;
    private double desiredRotationalAccelerationDegrees;
    private double desiredRotationDecelerationDegrees;
    private double angleCorrectionRange;
    private double maxRotationCorrectionVelocityDegrees;
    private double minRotationVelocityToMoveDegrees;
    private double maxToleranceDegrees;
    private double decelerationBufferDegrees;
    private boolean shouldMirror = false;
    

    /**
     * Constructs a {@code CMRotation} with detailed rotational motion
     * configuration and optional field mirroring.
     *
     * @param angleDegrees
     * The final angle of the robot in degrees.
     *
     * @param rotationDirection
     * The direction of the rotation: -1, 0, or 1.
     * <ul>
     *   <li>-1 = negative direction</li>
     *   <li>1 = positive direction</li>
     *   <li>0 = shortest path</li>
     * </ul>
     *
     * @param completeRotationPercent
     * Defines at what percentage of the trajectory the robot will stop trying
     * to reach this rotation.
     * <ul>
     *   <li>Lower bound: 0.0 (rotation completes immediately).</li>
     *   <li>Upper bound: 1.0 (rotation completes at the very end of the path).</li>
     * </ul>
     * <b>Note:</b> Only a value of 1.0 forces the robot to be at this rotation
     * when the trajectory ends.
     *
     * @param maxRotationVelocityDegrees
     * The maximum angular velocity the robot is allowed to rotate at.
     * <ul>
     *   <li>Suggested start: 240°/s</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: drivetrain’s safe max rotation speed</li>
     * </ul>
     *
     * @param desiredRotationalAccelerationDegrees
     * The angular acceleration used to increase rotation speed smoothly.
     * <ul>
     *   <li>Suggested start: 240°/s²</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: drivetrain’s safe rotational acceleration</li>
     * </ul>
     *
     * @param desiredRotationDecelerationDegrees
     * The angular deceleration used near the end of rotation to smoothly
     * reduce velocity until reaching the target angle.
     * <ul>
     *   <li>Suggested start: 240°/s²</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: drivetrain’s safe rotational deceleration</li>
     * </ul>
     *
     * @param angleCorrectionRange
     * The range (in degrees) around the target when the robot applies angle correction logic
     * <ul>
     *   <li>Suggested start: 1</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: Anything</li>
     * </ul>
     *
     * @param maxRotationCorrectionVelocityDegrees
     * The maximum angular velocity allowed when performing fine angle
     * corrections near the target.
     * <ul>
     *   <li>Suggested start: 5</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: must be ≤ {@code maxRotationVelocityDegrees}</li>
     * </ul>
     *
     * @param minRotationVelocityToMoveDegrees
     * The minimum angular velocity the robot will use when rotating. Prevents
     * the robot from stalling or crawling too slowly.
     * <ul>
     *   <li>Suggested start: .5</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: Depends on specfic robot. Likely close to .5</li>
     * </ul>
       *
     * @param decelerationBufferDegrees
     * The number of degrees before the target at which the robot begins
     * decelerating.
     * <ul>
     *   <li>Suggested start: 10°</li>
     *   <li>Lower bound: ≥ 0</li>
     *   <li>Upper bound: should not exceed full rotation angle</li>
     * </ul>
     *
     * @param maxTolorenceDegrees
     * The maximum allowed error in degrees for the rotation to be considered
     * complete. Inputed value is ±
     * <ul>
     *   <li>Suggested start: ±1°</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: depends on how precise the application must be</li>
     * </ul>
     * @param shouldMirror
     * Whether the direction and angle should be mirrored (e.g., for alliance
     * side switching).
     * <ul>
     *   <li>false = no mirroring</li>
     *   <li>true = mirror across the field centerline</li>
     * </ul>
     */
    public CMRotation(double angleDegrees,
        int rotationDirection, double completeRotationPercent, 
        double maxRotationVelocityDegrees, double desiredRotationalAccelerationDegrees, double desiredRotationDecelerationDegrees, 
        double angleCorrectionRange, double maxRotationCorrectionVelocityDegrees, double minRotationVelocityToMoveDegrees,
        double decelerationBufferDegrees,
        double maxTolorenceDegrees, 
        boolean shouldMirror) {
            
        this.angleDegrees = angleDegrees;
        this.rotationDirection = rotationDirection;
        this.completeRotationPercent = completeRotationPercent;
        this.maxRotationVelocityDegrees = maxRotationVelocityDegrees;
        this.desiredRotationalAccelerationDegrees = desiredRotationalAccelerationDegrees;
        this.desiredRotationDecelerationDegrees = desiredRotationDecelerationDegrees;
        this.angleCorrectionRange = angleCorrectionRange;
        this.maxRotationCorrectionVelocityDegrees = maxRotationCorrectionVelocityDegrees;
        this.minRotationVelocityToMoveDegrees = minRotationVelocityToMoveDegrees;
        this.maxToleranceDegrees = maxTolorenceDegrees;
        this.decelerationBufferDegrees = decelerationBufferDegrees;
        this.shouldMirror = shouldMirror;

        if(maxRotationVelocityDegrees <= 0) {
            throw new ExceptionInInitializerError(
                "CrowMotion Rotation's maximum rotation velocity must be positive, not: " + maxRotationVelocityDegrees
            );
        }

        if(desiredRotationalAccelerationDegrees <= 0) {
            throw new ExceptionInInitializerError(
                "CrowMotion Rotation's desired rotational acceleration must be positive, not: " + desiredRotationalAccelerationDegrees
            );
        }
        
        if(desiredRotationDecelerationDegrees <= 0) {
            throw new ExceptionInInitializerError(
                "CrowMotion Rotation's desired rotation deceleration must be positive, not: " + desiredRotationDecelerationDegrees
            );
        }
        
        if(angleCorrectionRange < 0) {
            throw new ExceptionInInitializerError(
                "CrowMotion Rotation's angle correction range must be >= 0, not: " + angleCorrectionRange
            );
        }
        
        if(decelerationBufferDegrees < 0) {
            throw new ExceptionInInitializerError(
                "CrowMotion Rotation's deceleration buffer degrees must be >= 0, not: " + decelerationBufferDegrees
            );
        }
        
        if(minRotationVelocityToMoveDegrees <= 0) {
            throw new ExceptionInInitializerError(
                "CrowMotion Rotation's minimum rotation velocity to move must be positive, not: " + minRotationVelocityToMoveDegrees
            );
        }
        
        if(maxRotationCorrectionVelocityDegrees <= 0) {
            throw new ExceptionInInitializerError(
                "CrowMotion Rotation's maximum rotation correction velocity must be positive, not: " + maxRotationCorrectionVelocityDegrees
            );
        }
        
        if(maxTolorenceDegrees <= 0) {
            throw new ExceptionInInitializerError(
                "CrowMotion Rotation's maximum tolerance degrees must be positive, not: " + maxTolorenceDegrees
            );
        }

        if(this.rotationDirection != 0 && this.rotationDirection != 1 && this.rotationDirection != -1) {
            throw new ExceptionInInitializerError(
                "CrowMotion Rotation's rotation direction must be 0, 1 or -1 not: " + this.rotationDirection
            );
        }

        if(this.completeRotationPercent < 0 || this.completeRotationPercent > 1) {
            throw new ExceptionInInitializerError(
                "CrowMotion Rotation's complete rotation percent between(inclusive) 0 and 1 not: " + this.completeRotationPercent
            );
        }
    }
    
    /**
     * Constructs a {@code CMRotation} with detailed rotational motion
     * configuration and optional field mirroring.
     *
     * @param angleDegrees
     * The final angle of the robot in degrees.
     *
     * @param rotationDirection
     * The direction of the rotation: -1, 0, or 1.
     * <ul>
     *   <li>-1 = negative direction</li>
     *   <li>1 = positive direction</li>
     *   <li>0 = shortest path</li>
     * </ul>
     *
     * @param completeRotationPercent
     * Defines at what percentage of the trajectory the robot will stop trying
     * to reach this rotation.
     * <ul>
     *   <li>Lower bound: 0.0 (rotation completes immediately).</li>
     *   <li>Upper bound: 1.0 (rotation completes at the very end of the path).</li>
     * </ul>
     * <b>Note:</b> Only a value of 1.0 forces the robot to be at this rotation
     * when the trajectory ends.
     *
     * @param maxRotationVelocityDegrees
     * The maximum angular velocity the robot is allowed to rotate at.
     * <ul>
     *   <li>Suggested start: 240°/s</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: drivetrain’s safe max rotation speed</li>
     * </ul>
     *
     * @param desiredRotationalAccelerationDegrees
     * The angular acceleration used to increase rotation speed smoothly.
     * <ul>
     *   <li>Suggested start: 240°/s²</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: drivetrain’s safe rotational acceleration</li>
     * </ul>
     *
     * @param desiredRotationDecelerationDegrees
     * The angular deceleration used near the end of rotation to smoothly
     * reduce velocity until reaching the target angle.
     * <ul>
     *   <li>Suggested start: 240°/s²</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: drivetrain’s safe rotational deceleration</li>
     * </ul>
     *
     * @param angleCorrectionRange
     * The range (in degrees) around the target when the robot applies angle correction logic
     * <ul>
     *   <li>Suggested start: 1</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: Anything</li>
     * </ul>
     *
     * @param maxRotationCorrectionVelocityDegrees
     * The maximum angular velocity allowed when performing fine angle
     * corrections near the target.
     * <ul>
     *   <li>Suggested start: 5</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: must be ≤ {@code maxRotationVelocityDegrees}</li>
     * </ul>
     *
     * @param minRotationVelocityToMoveDegrees
     * The minimum angular velocity the robot will use when rotating. Prevents
     * the robot from stalling or crawling too slowly.
     * <ul>
     *   <li>Suggested start: .5</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: Depends on specfic robot. Likely close to .5</li>
     * </ul>
       *
     * @param decelerationBufferDegrees
     * The number of degrees before the target at which the robot begins
     * decelerating.
     * <ul>
     *   <li>Suggested start: 10°</li>
     *   <li>Lower bound: ≥ 0</li>
     *   <li>Upper bound: should not exceed full rotation angle</li>
     * </ul>
     *
     * @param maxTolorenceDegrees
     * The maximum allowed error in degrees for the rotation to be considered
     * complete. Inputed value is ±
     * <ul>
     *   <li>Suggested start: ±1°</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: depends on how precise the application must be</li>
     * </ul>
     */
    public CMRotation(double angleDegrees,
        int rotationDirection, double completeRotationPercent, 
        double maxRotationVelocityDegrees, double desiredRotationalAccelerationDegrees, double desiredRotationDecelerationDegrees, 
        double angleCorrectionRange, double maxRotationCorrectionVelocityDegrees, double minRotationVelocityToMoveDegrees,
        double decelerationBufferDegrees,
        double maxTolorenceDegrees
        ) {
            this(angleDegrees,
            rotationDirection, completeRotationPercent, 
            maxRotationVelocityDegrees, desiredRotationalAccelerationDegrees,
            desiredRotationDecelerationDegrees, 
            angleCorrectionRange, maxRotationCorrectionVelocityDegrees,
            minRotationVelocityToMoveDegrees,
            decelerationBufferDegrees,
            maxTolorenceDegrees, CMConfig.shouldMirror());
    }

    /**
     * Constructs a {@code CMRotation} with detailed rotational motion
     * configuration and optional field mirroring.
     *
     * @param angleDegrees
     * The final angle of the robot in degrees.
     *
     * @param rotationDirection
     * The direction of the rotation: -1, 0, or 1.
     * <ul>
     *   <li>-1 = negative direction</li>
     *   <li>1 = positive direction</li>
     *   <li>0 = shortest path</li>
     * </ul>
     *
     * @param completeRotationPercent
     * Defines at what percentage of the trajectory the robot will stop trying
     * to reach this rotation.
     * <ul>
     *   <li>Lower bound: 0.0 (rotation completes immediately).</li>
     *   <li>Upper bound: 1.0 (rotation completes at the very end of the path).</li>
     * </ul>
     * <b>Note:</b> Only a value of 1.0 forces the robot to be at this rotation
     * when the trajectory ends.
     * 
     * @param angleCorrectionRange
     * The range (in degrees) around the target when the robot applies angle correction logic
     * <ul>
     *   <li>Suggested start: 1</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: Anything</li>
     * </ul>
     *
     * @param maxRotationCorrectionVelocityDegrees
     * The maximum angular velocity allowed when performing fine angle
     * corrections near the target.
     * <ul>
     *   <li>Suggested start: 5</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: must be ≤ {@code maxRotationVelocityDegrees}</li>
     * </ul>
     *
     * @param minRotationVelocityToMoveDegrees
     * The minimum angular velocity the robot will use when rotating. Prevents
     * the robot from stalling or crawling too slowly.
     * <ul>
     *   <li>Suggested start: .5</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: Depends on specfic robot. Likely close to .5</li>
     * </ul>
       *
     * @param decelerationBufferDegrees
     * The number of degrees before the target at which the robot begins
     * decelerating.
     * <ul>
     *   <li>Suggested start: 10°</li>
     *   <li>Lower bound: ≥ 0</li>
     *   <li>Upper bound: should not exceed full rotation angle</li>
     * </ul>
     *
     * @param maxTolorenceDegrees
     * The maximum allowed error in degrees for the rotation to be considered
     * complete. Inputed value is ±
     * <ul>
     *   <li>Suggested start: ±1°</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: depends on how precise the application must be</li>
     * </ul>
     */
    public CMRotation(double angleDegrees,
        int rotationDirection, double completeRotationPercent, 
        double angleCorrectionRange, double maxRotationCorrectionVelocityDegrees, double minRotationVelocityToMoveDegrees,
        double decelerationBufferDegrees,
        double maxTolorenceDegrees
        ) {
            this(angleDegrees,
            rotationDirection, completeRotationPercent, 
            CMConfig.getDefaultMaxDesiredRotationalVelocity(), CMConfig.getDefaultMaxDesiredRotationalAcceleration(),
            CMConfig.getDefaultMaxDesiredRotationalDeceleration(), 
            angleCorrectionRange, maxRotationCorrectionVelocityDegrees,
            minRotationVelocityToMoveDegrees,
            decelerationBufferDegrees,
            maxTolorenceDegrees, CMConfig.shouldMirror());
    }

    /**
     * Constructs a {@code CMRotation} with detailed rotational motion
     * configuration and optional field mirroring.
     *
     * @param angleDegrees
     * The final angle of the robot in degrees.
     *
     * @param rotationDirection
     * The direction of the rotation: -1, 0, or 1.
     * <ul>
     *   <li>-1 = negative direction</li>
     *   <li>1 = positive direction</li>
     *   <li>0 = shortest path</li>
     * </ul>
     *
     * @param completeRotationPercent
     * Defines at what percentage of the trajectory the robot will stop trying
     * to reach this rotation.
     * <ul>
     *   <li>Lower bound: 0.0 (rotation completes immediately).</li>
     *   <li>Upper bound: 1.0 (rotation completes at the very end of the path).</li>
     * </ul>
     * <b>Note:</b> Only a value of 1.0 forces the robot to be at this rotation
     * when the trajectory ends.
     * 
     * @param angleCorrectionRange
     * The range (in degrees) around the target when the robot applies angle correction logic
     * <ul>
     *   <li>Suggested start: 1</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: Anything</li>
     * </ul>
     *
     * @param maxRotationCorrectionVelocityDegrees
     * The maximum angular velocity allowed when performing fine angle
     * corrections near the target.
     * <ul>
     *   <li>Suggested start: 5</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: must be ≤ {@code maxRotationVelocityDegrees}</li>
     * </ul>
     *
     * @param minRotationVelocityToMoveDegrees
     * The minimum angular velocity the robot will use when rotating. Prevents
     * the robot from stalling or crawling too slowly.
     * <ul>
     *   <li>Suggested start: .5</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: Depends on specfic robot. Likely close to .5</li>
     * </ul>
       *
     * @param decelerationBufferDegrees
     * The number of degrees before the target at which the robot begins
     * decelerating.
     * <ul>
     *   <li>Suggested start: 10°</li>
     *   <li>Lower bound: ≥ 0</li>
     *   <li>Upper bound: should not exceed full rotation angle</li>
     * </ul>
     *
     * @param maxTolorenceDegrees
     * The maximum allowed error in degrees for the rotation to be considered
     * complete. Inputed value is ±
     * <ul>
     *   <li>Suggested start: ±1°</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: depends on how precise the application must be</li>
     * </ul>
     * @param shouldMirror
     * Whether the direction and angle should be mirrored (e.g., for alliance
     * side switching).
     * <ul>
     *   <li>false = no mirroring</li>
     *   <li>true = mirror across the field centerline</li>
     * </ul>
     */
    public CMRotation(double angleDegrees,
        int rotationDirection, double completeRotationPercent, 
        double angleCorrectionRange, double maxRotationCorrectionVelocityDegrees, double minRotationVelocityToMoveDegrees,
        double decelerationBufferDegrees,
        double maxTolorenceDegrees, boolean shouldMirror
        ) {
            this(angleDegrees,
            rotationDirection, completeRotationPercent, 
            CMConfig.getDefaultMaxDesiredRotationalVelocity(), CMConfig.getDefaultMaxDesiredRotationalAcceleration(),
            CMConfig.getDefaultMaxDesiredRotationalDeceleration(), 
            angleCorrectionRange, maxRotationCorrectionVelocityDegrees,
            minRotationVelocityToMoveDegrees,
            decelerationBufferDegrees,
            maxTolorenceDegrees, shouldMirror);
    }

    /**
     * Constructs a {@code CMRotation} with detailed rotational motion
     * configuration and optional field mirroring.
     *
     * @param angleDegrees
     * The final angle of the robot in degrees.
     *
     * @param rotationDirection
     * The direction of the rotation: -1, 0, or 1.
     * <ul>
     *   <li>-1 = negative direction</li>
     *   <li>1 = positive direction</li>
     *   <li>0 = shortest path</li>
     * </ul>
     *
     * @param completeRotationPercent
     * Defines at what percentage of the trajectory the robot will stop trying
     * to reach this rotation.
     * <ul>
     *   <li>Lower bound: 0.0 (rotation completes immediately).</li>
     *   <li>Upper bound: 1.0 (rotation completes at the very end of the path).</li>
     * </ul>
     * <b>Note:</b> Only a value of 1.0 forces the robot to be at this rotation
     * when the trajectory ends.
     *
     * @param maxRotationVelocityDegrees
     * The maximum angular velocity the robot is allowed to rotate at.
     * <ul>
     *   <li>Suggested start: 240°/s</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: drivetrain’s safe max rotation speed</li>
     * </ul>
     *
     * @param desiredRotationalAccelerationDegrees
     * The angular acceleration used to increase rotation speed smoothly.
     * <ul>
     *   <li>Suggested start: 240°/s²</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: drivetrain’s safe rotational acceleration</li>
     * </ul>
     *
     * @param desiredRotationDecelerationDegrees
     * The angular deceleration used near the end of rotation to smoothly
     * reduce velocity until reaching the target angle.
     * <ul>
     *   <li>Suggested start: 240°/s²</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: drivetrain’s safe rotational deceleration</li>
     * </ul>
     * @param maxTolorenceDegrees
     * The maximum allowed error in degrees for the rotation to be considered
     * complete. Inputed value is ±
     * <ul>
     *   <li>Suggested start: ±1°</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: depends on how precise the application must be</li>
     * </ul>
     */
    public CMRotation(double angleDegrees,
        int rotationDirection, double completeRotationPercent, 
        double maxRotationVelocityDegrees, double desiredRotationalAccelerationDegrees, double desiredRotationDecelerationDegrees, 
        double maxTolorenceDegrees
        ) {
            this(angleDegrees,
            rotationDirection, completeRotationPercent, 
            maxRotationVelocityDegrees, desiredRotationalAccelerationDegrees,
            desiredRotationDecelerationDegrees, 
            CMConfig.getDefaultAngleCorrectionRange(), CMConfig.getDefaultMaxRotationCorrectionVelocityDegrees(),
            CMConfig.getDefaultMinRotationVelocityToMove(),
            CMConfig.getDefaultDecelerationBufferDegrees(),
            maxTolorenceDegrees, CMConfig.shouldMirror());
    }

    /**
     * Constructs a {@code CMRotation} with detailed rotational motion
     * configuration and optional field mirroring.
     *
     * @param angleDegrees
     * The final angle of the robot in degrees.
     *
     * @param rotationDirection
     * The direction of the rotation: -1, 0, or 1.
     * <ul>
     *   <li>-1 = negative direction</li>
     *   <li>1 = positive direction</li>
     *   <li>0 = shortest path</li>
     * </ul>
     *
     * @param completeRotationPercent
     * Defines at what percentage of the trajectory the robot will stop trying
     * to reach this rotation.
     * <ul>
     *   <li>Lower bound: 0.0 (rotation completes immediately).</li>
     *   <li>Upper bound: 1.0 (rotation completes at the very end of the path).</li>
     * </ul>
     * <b>Note:</b> Only a value of 1.0 forces the robot to be at this rotation
     * when the trajectory ends.
     *
     * @param maxRotationVelocityDegrees
     * The maximum angular velocity the robot is allowed to rotate at.
     * <ul>
     *   <li>Suggested start: 240°/s</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: drivetrain’s safe max rotation speed</li>
     * </ul>
     *
     * @param desiredRotationalAccelerationDegrees
     * The angular acceleration used to increase rotation speed smoothly.
     * <ul>
     *   <li>Suggested start: 240°/s²</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: drivetrain’s safe rotational acceleration</li>
     * </ul>
     *
     * @param desiredRotationDecelerationDegrees
     * The angular deceleration used near the end of rotation to smoothly
     * reduce velocity until reaching the target angle.
     * <ul>
     *   <li>Suggested start: 240°/s²</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: drivetrain’s safe rotational deceleration</li>
     * </ul>
     * @param maxTolorenceDegrees
     * The maximum allowed error in degrees for the rotation to be considered
     * complete. Inputed value is ±
     * <ul>
     *   <li>Suggested start: ±1°</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: depends on how precise the application must be</li>
     * </ul>
     * @param shouldMirror
     * Whether the direction and angle should be mirrored (e.g., for alliance
     * side switching).
     * <ul>
     *   <li>false = no mirroring</li>
     *   <li>true = mirror across the field centerline</li>
     * </ul>
     */
    public CMRotation(double angleDegrees,
        int rotationDirection, double completeRotationPercent, 
        double maxRotationVelocityDegrees, double desiredRotationalAccelerationDegrees, double desiredRotationDecelerationDegrees, 
        double maxTolorenceDegrees, boolean shouldMirror
        ) {
            this(angleDegrees,
            rotationDirection, completeRotationPercent, 
            maxRotationVelocityDegrees, desiredRotationalAccelerationDegrees,
            desiredRotationDecelerationDegrees, 
            CMConfig.getDefaultAngleCorrectionRange(), CMConfig.getDefaultMaxRotationCorrectionVelocityDegrees(),
            CMConfig.getDefaultMinRotationVelocityToMove(),
            CMConfig.getDefaultDecelerationBufferDegrees(),
            maxTolorenceDegrees, shouldMirror);
    }

    /**
     * Constructs a {@code CMRotation} with detailed rotational motion
     * configuration and optional field mirroring.
     *
     * @param angleDegrees
     * The final angle of the robot in degrees.
     *
     * @param rotationDirection
     * The direction of the rotation: -1, 0, or 1.
     * <ul>
     *   <li>-1 = negative direction</li>
     *   <li>1 = positive direction</li>
     *   <li>0 = shortest path</li>
     * </ul>
     *
     * @param completeRotationPercent
     * Defines at what percentage of the trajectory the robot will stop trying
     * to reach this rotation.
     * <ul>
     *   <li>Lower bound: 0.0 (rotation completes immediately).</li>
     *   <li>Upper bound: 1.0 (rotation completes at the very end of the path).</li>
     * </ul>
     * <b>Note:</b> Only a value of 1.0 forces the robot to be at this rotation
     * when the trajectory ends.
     * @param maxTolorenceDegrees
     * The maximum allowed error in degrees for the rotation to be considered
     * complete. Inputed value is ±
     * <ul>
     *   <li>Suggested start: ±1°</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: depends on how precise the application must be</li>
     * </ul>
     * @param shouldMirror
     * Whether the direction and angle should be mirrored (e.g., for alliance
     * side switching).
     * <ul>
     *   <li>false = no mirroring</li>
     *   <li>true = mirror across the field centerline</li>
     * </ul>
     */
    public CMRotation(double angleDegrees,
        int rotationDirection, double completeRotationPercent, 
        double maxTolorenceDegrees, boolean shouldMirror
        ) {
            this(angleDegrees,
            rotationDirection, completeRotationPercent, 
            CMConfig.getDefaultMaxDesiredRotationalVelocity(), CMConfig.getDefaultMaxDesiredRotationalAcceleration(),
            CMConfig.getDefaultMaxDesiredRotationalDeceleration(), 
            CMConfig.getDefaultAngleCorrectionRange(), CMConfig.getDefaultMaxRotationCorrectionVelocityDegrees(),
            CMConfig.getDefaultMinRotationVelocityToMove(),
            CMConfig.getDefaultDecelerationBufferDegrees(),
            maxTolorenceDegrees, shouldMirror);
    }

    /**
     * Constructs a {@code CMRotation} with detailed rotational motion
     * configuration and optional field mirroring.
     *
     * @param angleDegrees
     * The final angle of the robot in degrees.
     *
     * @param rotationDirection
     * The direction of the rotation: -1, 0, or 1.
     * <ul>
     *   <li>-1 = negative direction</li>
     *   <li>1 = positive direction</li>
     *   <li>0 = shortest path</li>
     * </ul>
     *
     * @param completeRotationPercent
     * Defines at what percentage of the trajectory the robot will stop trying
     * to reach this rotation.
     * <ul>
     *   <li>Lower bound: 0.0 (rotation completes immediately).</li>
     *   <li>Upper bound: 1.0 (rotation completes at the very end of the path).</li>
     * </ul>
     * <b>Note:</b> Only a value of 1.0 forces the robot to be at this rotation
     * when the trajectory ends.
     * @param maxTolorenceDegrees
     * The maximum allowed error in degrees for the rotation to be considered
     * complete. Inputed value is ±
     * <ul>
     *   <li>Suggested start: ±1°</li>
     *   <li>Lower bound: > 0</li>
     *   <li>Upper bound: depends on how precise the application must be</li>
     * </ul>
     */
    public CMRotation(double angleDegrees,
        int rotationDirection, double completeRotationPercent, 
        double maxTolorenceDegrees
        ) {
            this(angleDegrees,
            rotationDirection, completeRotationPercent, 
            CMConfig.getDefaultMaxDesiredRotationalVelocity(), CMConfig.getDefaultMaxDesiredRotationalAcceleration(),
            CMConfig.getDefaultMaxDesiredRotationalDeceleration(), 
            CMConfig.getDefaultAngleCorrectionRange(), CMConfig.getDefaultMaxRotationCorrectionVelocityDegrees(),
            CMConfig.getDefaultMinRotationVelocityToMove(),
            CMConfig.getDefaultDecelerationBufferDegrees(),
            maxTolorenceDegrees, CMConfig.shouldMirror());
    }
    
    /**
     * Gets the angle of rotation in degrees.
     *
     * @return the angle in degrees
     */
    public double getAngleDegrees() {
        return wrapAngle(shouldMirror ? this.angleDegrees * -1 : this.angleDegrees);
    }

    /**
     * Gets the direction of rotation.
     *
     * @return the rotation direction as a RotationDirrection enum
     */
    public int getRotationDirection() {
        return shouldMirror ? (rotationDirection == 1 ? -1 : 1)
        : rotationDirection;
    }

    /**
     * Gets the percent of the path by which rotation should be completed.
     *
     * @return the percent (0.0 to 1.0)
     */
    public double getCompleteRotationPercent() {
        return this.completeRotationPercent;
    }

    public double getMaxRotationVelocityDegrees() {
        return maxRotationVelocityDegrees;
    }
    
    public double getDesiredRotationalAccelerationDegrees() {
        return desiredRotationalAccelerationDegrees;
    }
    
    public double getDesiredRotationalDecelerationDegrees() {
        return desiredRotationDecelerationDegrees;
    }

    public double getAngleCorrectionRange() {
        return this.angleCorrectionRange;
    }

    public static double wrapAngle(double angleDegrees) {
        return (angleDegrees + 180) % 360 - 180;
    }

    public double getMaxRotationCorrectionVelocityDegrees() {
        return this.maxRotationCorrectionVelocityDegrees;
    }

    public double getMinRotationVelocityToMoveDegrees() {
        return this.minRotationVelocityToMoveDegrees;
    }

    public double getMaxToleranceDegrees() {
        return this.maxToleranceDegrees;
    }

    public double getDecelerationBufferDegrees() {
        return this.decelerationBufferDegrees;
    }
}
